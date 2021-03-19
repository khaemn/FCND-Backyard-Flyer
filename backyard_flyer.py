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

class Dir(Enum):
    LON=0
    LAT=1
    ALT=2
    
# Trajectory is square with the given side lenght (m)
SQUARE_SIDE_M = 10.0

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

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.waypoints = []

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_phase == Phases.TAKEOFF:

            # coordinate conversion 
            current_position = np.array([0.0, 0.0, 0.0])
            current_position[DIR.ALT] = -1.0 * self.local_position[DIR.ALT]
            current_position[DIR.LAT] = self.local_position[DIR.LAT]
            current_position[DIR.LON] = self.local_position[DIR.LON]

            # check if altitude is within 95% of target
            waypoint_precision_m = 0.1
            target_reached = True
            
            for axis in [DIR.LAT, DIR.LON, DIR.ALT]:
                diff = abs(self.target_position[axis] - current_position[axis])
                target_reached &= (diff < waypoint_precision)
                
            if target_reached:
                # TODO: go to waypoint traversal here!
                self.landing_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_phase == Phases.LANDING:
            if ((self.global_position[DIR.ALT] - self.global_home[DIR.ALT] < 0.1) and
            abs(self.local_position[DIR.ALT]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_phase == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == Phases.ARMING:
            self.takeoff_transition()
        elif self.flight_phase == Phases.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        # To generate a square waypoints, I make 4 of them:
        # 1. Home + side_lenght along LON
        # 2. p.1 + side_lenght along LAT
        # 3. p.2 - side_lenght along LON
        # 4. p.3 - side_lenght along LAT (== Home)
        # Note: ALT direction is always the same here and not taken into account.
        steps = [
            [SQUARE_SIDE_M, DIR.LON],
            [SQUARE_SIDE_M, DIR.LAT],
            [-SQUARE_SIDE_M, DIR.LON],
            [-SQUARE_SIDE_M, DIR.LAT]
        ]
        
        current_point = [self.home_position[DIR.LON],
                         self.home_position[DIR.LAT],
                         self.mission_altitude]
        
        for distance, moving_direction in steps:
            current_point[moving_direction] += distance
            self.waypoints.append(current_point)
            
        # After all the steps, the last wpt must be at the home position
        for check_dir in [DIR.LON, DIR.LAT]:
            assert current_point[check_dir] == self.home_position[check_dir]
            
        print("self.waypoints: ", self.waypoints)

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[DIR.LON],
                               self.global_position[DIR.LAT],
                               self.global_position[DIR.ALT])
        
        # The mission waypoints can be calculated before the takeoff,
        # but after the vehicle knows its home position.
        self.calculate_box()
        
        self.flight_phase = Phases.ARMING
        print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        self.target_position[DIR.ALT] = self.mission_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
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
