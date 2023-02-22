#! /usr/bin/env python

"""
 * File: offb_node.py
 * Stack and tested in Gazebo 11 SITL
"""

import numpy as np
import numpy.linalg as la

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class OffboardNode:
    def __init__(self):

        rospy.init_node("offboard_node", log_level=rospy.INFO)
        rospy.loginfo("Starting OffboardNode.")

        # Create subscriber to check current state autopilot
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)

        # Create publisher to publish commanded local position
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Create subscriber to retrieve current local position
        self.odom_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_pos_callback)  # EKF

        # Create client to request arming
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        # Create client to request mode change
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Create client to request landing
        self.landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

        # PX4 has a 500ms timeout between 2 offboard commands
        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        # Define trajectory, last position is the position to prepare for landing, will move to safe position if detect
        # human
        self.trajectory = np.array([[0.0, 0.0, 4.5]])
        self.pos_b4_landing = np.array([0, 0, 0.5])
        self.num_of_pos = len(self.trajectory)  # total number of poses in trajectory

        # Initialize current state
        self.current_state = State()

        # Initialize current position and target to (0, 0, 0)
        self.current_local_pos = np.zeros(3)
        self.target_pos = np.zeros(3)

        # Define counter to determine at which pose in the trajectory we're at
        self.count = 0

        # Define threshold to determine when next position in trajectory is reached
        self.threshold = 0.2
        self.landing_threshold = 0.01

        # Define parameter to choose how fast to move to next position
        self.speed_param = 0.7

        # Define safety parameter for takeoff and landing
        self.emergency = False

        # Will hover at this position if human is detected
        self.safe_position = np.array([0, 0, 3])
        self.landed = False

    def state_callback(self, msg):
        """
        Callback to save the current state of the autopilot.
        This will allow us to check connection, arming and OFFBOARD flags.
        """
        self.current_state = msg

    def current_pos_callback(self, msg):
        """
        Callback to retrieve current local position
        """
        current_pos = msg.pose.position
        self.current_local_pos = np.array([current_pos.x, current_pos.y, current_pos.z])

    def wait_for_FCU_connection(self):
        """
        Function check connection between MAVROS and autopilot
        """
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    def publish_setpoint(self, x, y, z):
        """
        Function to determine trajectory pose
        PX4 operates in the aerospace NED coordinate frame,
        MAVROS translates these coordinates to the standard ENU frame
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.local_pos_pub.publish(pose)

    def position_reached(self, target_pos):
        """
        Define function to check that distance between
        current position and next step lower than threshold
        """
        return la.norm(target_pos - self.current_local_pos) <= self.threshold

    def get_next_pos(self):
        """
        Define function that moves between two setpoints
        """

        # Check that there are steps to take
        next_pos = None
        # # Get next target position
        # current_pos = self.current_local_pos.copy()
        # if self.count < self.num_of_pos:
        #     # Retrieve next final target position
        #     self.target_pos = self.trajectory[self.count]
        #     # Check if current position is close to next step
        #     if self.position_reached(self.target_pos):
        #         rospy.loginfo(f"Reached position: {self.trajectory[self.count]}")
        #         # Increment step counter
        #         self.count += 1
        #         if self.count < self.num_of_pos:
        #             rospy.loginfo(f"Moving towards position: {self.trajectory[self.count]}")
        #         else:
        #             rospy.loginfo(f"Preparing for landing. Moving towards position: {self.pos_b4_landing}")
        # elif not self.position_reached(self.pos_b4_landing):
        #     self.target_pos = self.pos_b4_landing
        # else:
        #     self.target_pos = None
        #
        # if self.target_pos is not None:
        #     # Compute vector between current position and next target position
        #     pos_targ_vector = self.target_pos - current_pos
        #
        #     # Compute unit vector determining direction
        #     unit_vector = pos_targ_vector / la.norm(pos_targ_vector)
        #
        #     # Compute next incremental step in desired direction
        #     next_pos = current_pos + self.speed_param * unit_vector

        # # Use this if you don't need to fly smoothly (prob better for real flights)
        # Get next target position
        if self.count < self.num_of_pos:
            next_pos = self.trajectory[self.count]
            # Check if current position is close to next step
            if self.position_reached(self.trajectory[self.count]):
                rospy.loginfo(f"Reached position: {self.trajectory[self.count]}")
                # Increment step counter
                self.count += 1
                if self.count < self.num_of_pos:
                    rospy.loginfo(f"Moving towards position: {self.trajectory[self.count]}")
                else:
                    rospy.loginfo(f"Preparing for landing. Moving towards position: {self.pos_b4_landing}")
        elif not self.position_reached(self.pos_b4_landing):
            next_pos = self.pos_b4_landing
        else:
            next_pos = None

        self.target_pos = next_pos

        return next_pos

    # Define function to land drone
    def land(self):
        rospy.wait_for_service("mavros/cmd/land")
        if self.landing_client().success:
            rospy.loginfo("Landing vehicle")

    def fly(self):
        """
        Function that flies the drone to every point in the predefined
        """
        # Wait for connection between MAVROS and autopilot to be established (Flight Controller connection)
        rospy.loginfo("Getting ready...")
        self.wait_for_FCU_connection()
        rospy.loginfo("Done!")

        # It is recommended to enter OFFBOARD mode from Position mode, this way if the vehicle drops out of OFFBOARD
        # mode it will stop in its tracks and hover.
        # For Gazebo simulation, first check that the setpoints are being sent, then arm the vehicle with
        # "commander arm", then switch to offboard mode with "commander mode offboard"

        next_pos = None
        while not rospy.is_shutdown():
            if self.current_state.mode != "AUTO.LAND":
                # Retrieve next position to move to
                next_pos = self.get_next_pos()

            # If a person is detected and is nearby, hover at safe position
            if self.emergency:
                next_pos = self.safe_position

            # Check that we haven't reached end of trajectory (or next_pos isn't None)
            if next_pos is not None:
                # Move to next pose on trajectory
                self.publish_setpoint(next_pos[0], next_pos[1], next_pos[2])

            # If we reached the end of the trajectory, land
            if self.count >= self.num_of_pos and self.position_reached(self.pos_b4_landing):
                self.land()
                if self.current_local_pos[2] <= self.landing_threshold and self.landed:
                    rospy.loginfo("Landed vehicle, bye!")
                    self.landed = True
            self.rate.sleep()  # sleeps long enough to maintain desired rate


if __name__ == "__main__":

    offboard_node = OffboardNode()
    offboard_node.fly()
