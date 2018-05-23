#!/usr/bin/env python
import argparse
import time
from enum import Enum
from collections import deque
from math import pi

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

class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = None
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.position_or_velocity_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.position_or_velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    @property
    def local_speed(self):
        return np.linalg.norm(self.local_velocity)
    
    @property
    def distance_to_target_position(self):
        return np.linalg.norm(self.target_position[:3] - self.local_position)

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            if self.flight_state == States.MANUAL and not self.armed and not self.guided:
                self.arming_transition()

            if self.flight_state == States.ARMING and     self.armed and     self.guided:
                self.takeoff_transition()

            if self.flight_state == States.DISARMING and not self.armed:
                self.manual_transition()

    def position_or_velocity_callback(self):
        if self.in_mission:
            if self.flight_state is States.TAKEOFF and self.takeoff_is_complete() and self.all_waypoints:
                self.waypoint_transition()

            if self.flight_state is States.WAYPOINT and self.arrived_at_waypoint():
                if self.all_waypoints:
                    self.waypoint_transition()
                else:
                    self.landing_transition()

            if self.flight_state is States.LANDING and self.landing_is_complete():
                self.disarming_transition()

    def takeoff_is_complete(self):
        return self.local_speed < 0.1 and -self.local_position[2] > 2.9
    
    def arrived_at_waypoint(self):
        return self.local_speed < 0.1 and self.distance_to_target_position < 0.15
        
    def landing_is_complete(self):
        return self.local_speed < 0.1 and -self.local_position[2] < 0.1
        
    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        return deque([
            np.array([10, 0,  -3, pi/2]),
            np.array([10, 10, -3, pi]),
            np.array([0,  10, -3, 3*pi/2]),
            np.array([0,  0,  -3, 0])
        ])

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(*self.global_position)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        print("local position:")
        print(self.local_position)
        self.takeoff(3)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.all_waypoints.popleft()
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            -self.target_position[2],
            self.target_position[3])
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        drone.disarm()
        self.flight_state = States.DISARMING

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
