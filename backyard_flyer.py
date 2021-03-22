import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

DIR_LON=0
DIR_LAT=1
DIR_ALT=2
    
# Trajectory is square with the given side lenght (m)
SQUARE_SIDE_M = 30.0

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.mission_altitude = 3.0
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        
        self.waypoint_precision_m = 0.2
        self.min_eligible_altitude_m = 0.02
        self.waypoints = []
        self.wpt_index = 0

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state in [States.TAKEOFF, States.WAYPOINT]:
            self.waypoint_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            diff = abs(self.global_position[DIR_ALT] - self.global_home[DIR_ALT])
            is_at_home = diff < self.waypoint_precision_m
            is_at_low_altitude = self.local_position[DIR_ALT] < self.min_eligible_altitude_m
            if is_at_home and is_at_low_altitude:
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        print("Current state: ", self.flight_state)
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.WAYPOINT:
            self.waypoint_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """
        1. Returns waypoints to fly a box
        """
        # To generate a square waypoints, I make 4 of them:
        # 1. Home + side_lenght along LON
        # 2. p.1 + side_lenght along LAT
        # 3. p.2 - side_lenght along LON
        # 4. p.3 - side_lenght along LAT (== Home)
        # Note: ALT direction is always the same here and not taken into account.
        steps = [
            [SQUARE_SIDE_M, DIR_LON],
            [SQUARE_SIDE_M, DIR_LAT],
            [-SQUARE_SIDE_M, DIR_LON],
            [-SQUARE_SIDE_M, DIR_LAT]
        ]
        
        current_point = np.array([self._home_longitude,
                                  self._home_latitude,
                                  self.mission_altitude])
        waypoints = []
        for distance, moving_direction in steps:
            new_point = current_point.copy()
            new_point[moving_direction] += distance
            waypoints.append(new_point)
            current_point = new_point
            
        print("Generated waypoints: ", waypoints)

        # After all the steps, the last wpt must be at the home position
        assert abs(current_point[DIR_LAT] - self._home_latitude) < self.waypoint_precision_m
        assert abs(current_point[DIR_LON] - self._home_longitude) < self.waypoint_precision_m
        return waypoints

    def arming_transition(self):
        """
        1. Takes control of the drone
        2. Passes an arming command
        3. Sets the home location to current position
        4. Transitions to the ARMING state
        """
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[DIR_LON],
                               self.global_position[DIR_LAT],
                               self.global_position[DIR_ALT])
        
        # The mission waypoints can be calculated before the takeoff,
        # but after the vehicle knows its home position.
        self.waypoints = self.calculate_box()
        
        self.flight_state = States.ARMING
        print("arming transition")

    def takeoff_transition(self):
        """
        1. Sets target_position altitude to self.mission_altitude
        2. Commands a takeoff to self.mission_altitude
        3. Transitions to the TAKEOFF state
        """
        self.target_position[DIR_ALT] = self.mission_altitude
        self.takeoff(self.mission_altitude)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        """
        1. Commands the next waypoint position
        2. Transitions to WAYPOINT state
        """
        # coordinate conversion 
        current_position = np.array([0.0, 0.0, 0.0])
        current_position[DIR_ALT] = -1.0 * self.local_position[DIR_ALT]
        current_position[DIR_LAT] = self.local_position[DIR_LAT]
        current_position[DIR_LON] = self.local_position[DIR_LON]

        target_reached = True

        for axis in [DIR_LAT, DIR_LON, DIR_ALT]:
            diff = abs(self.target_position[axis] - current_position[axis])
            target_reached &= (diff < self.waypoint_precision_m)

        if target_reached:
            if self.wpt_index >= len(self.waypoints):
                self.landing_transition()
                return
            self.target_position = self.waypoints[self.wpt_index]
            self.cmd_position(self.target_position[0],
                              self.target_position[1],
                              self.target_position[2],
                              0.0)
            self.wpt_index += 1
            self.flight_state = States.WAYPOINT

        print("waypoint transition")

    def landing_transition(self):
        """
        1. Commands the drone to land
        2. Transitions to the LANDING state
        """
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """
        1. Commands the drone to disarm
        2. Transitions to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """
        1. Releases control of the drone
        2. Stops the connection (and telemetry log)
        3. Ends the mission
        4. Transitions to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        1. Opens a log file
        2. Starts the drone connection
        3. Closes the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
