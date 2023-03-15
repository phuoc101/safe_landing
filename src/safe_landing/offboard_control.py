"""
 * File: offb_node.py
 * Stack and tested in Gazebo 11 SITL
"""

import numpy as np
import numpy.linalg as la

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from detection_msgs.msg import BoundingBoxesDist
from safe_landing.flight_mode import FlightMode


class OffboardNode:
    def __init__(self):
        # Set up ROS
        self.ros_setup()

        # Define counter to determine at which pose in the trajectory we're at
        self.count = 0
        # Define threshold to determine when next position in trajectory is reached
        self.threshold = 0.2
        self.landing_threshold = 0.01

        # Define mission trajectory
        self.trajectory = np.array([[0.0, 0.0, 4.5]])
        self.num_of_pos = len(self.trajectory)  # total number of poses in trajectory

        # Define safety parameters for landing (TODO: For takeoff as well)
        # position to move to before auto landing
        self.prelanding_pos = np.array([0, 0, 1])
        # Will hover at this position if human is detected
        self.safe_position = np.array([0, 0, 3])
        # If some people is in close vicinity, don't land (TODO: animal lives matter, but need more data)
        self.safe_dist = 2
        self.targets_to_avoid = ["person"]
        # land after enough msgs without danger
        self.safe_counter_threshold = 20

        # Define FLAGS
        self.flight_mode = FlightMode.PRE_TAKEOFF
        self.OFFBOARD = False  # Flag turns on when flight mode is offboard
        self.ARMED = False  # Flag turns on when drone is armed
        self.EMERGENCY = True  # Flag turns on when danger detected
        self.LANDED = False  # Flag turns on when mission is completed
        self.safe_counter = 0

    def ros_setup(self):
        """
        Function to set up ROS:
            - Init ROS Node
            - Set up PX4/MAVROS
            - Set up ROS communication with object detector
            - Set up ROS parameters
        """
        rospy.init_node("offboard_node", log_level=rospy.INFO)
        rospy.loginfo("Starting OffboardNode.")

        # PX4/MAVROS
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        self.local_setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_pos_callback)
        self.landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

        # YOLO Detector
        self.detection_topic = rospy.get_param("~detection_topic", "/yolov7/detections_dist")
        self.detection_sub = rospy.Subscriber(self.detection_topic, BoundingBoxesDist, self.detection_cb)

        # PX4 has a 500ms timeout between 2 offboard commands
        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)
        self.current_state = State()
        # Initialize current position and target to (0, 0, 0)
        self.current_local_pos = np.zeros(3)

    def state_callback(self, msg):
        """
        Callback to save the current state of the autopilot.
        This will allow us to check connection, arming and OFFBOARD flags.
        """
        self.current_state = msg
        if not msg.armed:
            self.ARMED = False
        elif not self.ARMED:
            rospy.loginfo("Vehicle armed")
            self.ARMED = True
        if msg.mode != "OFFBOARD":
            self.OFFBOARD = False
        elif not self.OFFBOARD:
            rospy.loginfo("Vehicle in offboard mode")
            self.OFFBOARD = True

    def current_pos_callback(self, msg):
        """
        Callback to retrieve current local position
        """
        current_pos = msg.pose.position
        self.current_local_pos = np.array([current_pos.x, current_pos.y, current_pos.z])

    def detection_cb(self, msg):
        """
        Callback to retrieve and handle hazard detection (currently for landing only.)
        TODO: do the same for takeoff as well
        """
        is_safe = True
        for box in msg.bounding_boxes:
            if box.Class in self.targets_to_avoid and box.dist <= self.safe_dist:
                self.safe_counter = 0
                self.EMERGENCY = True
                is_safe = False
                break
        if is_safe:
            self.safe_counter += 1
            if self.safe_counter >= self.safe_counter_threshold:
                self.safe_counter = 0
                self.EMERGENCY = False

    def wait_for_FCU_connection(self):
        """
        Function check connection between MAVROS and autopilot
        """
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    def wait_for_posctl_mode(self):
        """
        Function wait until drone is in POSCTL mode
        """
        while self.current_state.mode != "POSCTL":
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
        self.local_setpoint_pub.publish(pose)

    def position_reached(self, target_pos):
        """
        Function to check that distance between
        current position and next step lower than threshold
        """
        return la.norm(target_pos - self.current_local_pos) <= self.threshold

    def get_next_pos(self):
        """
        Function to get next position in mission trajectory. Update once vehicle gets close to waypoint (how close
        depends on self.position_reached())
        """
        next_pos = self.trajectory[self.count]
        if self.position_reached(self.trajectory[self.count]):
            rospy.loginfo(f"Reached position: {self.trajectory[self.count]}")
            # Increment step counter
            self.count += 1

        return next_pos

    def land(self):
        """
        Function to call MAVROS landing service
        """
        rospy.wait_for_service("mavros/cmd/land")
        if self.landing_client().success:
            rospy.loginfo("Landing vehicle")

    def fly(self):
        """
        Function that flies the drone to every point in the predefined

        It is recommended to enter OFFBOARD mode from Position mode (especially in real flights), this way if the
        vehicle drops out of OFFBOARD mode it will stop in its tracks and hover.

        For Gazebo simulation, first check that the setpoints are being sent, then arm the vehicle with
        "commander arm", then switch to offboard mode with "commander mode offboard"

        Before mission begin, check for vehicle connection and that the vehicle is in POSCTL mode
        Mode handling:
            - PRE_TAKEOFF: Switch to MISSION mode if area is clear, else just stay in same place without publishing any
            way point. This should prevent OFFBOARD mode from being switched into.
            - MISSION: Fly according to mission trajectory, once finish mission, switch to PRE_LANDING
            - PRE_LANDING: Go to prelanding position, if any danger detected, switch to EMERGENCY_RETREAT, once at
            prelanding position, switch to full LANDING
            - LANDING: Safe to land, call MAVROS service to land and end mission
            - EMERGENCY_RETREAT: retreat and hover at safe position, once area is clear again, switch back to
            PRE_LANDING
        """
        rospy.loginfo("Getting ready...")
        rospy.loginfo("Waiting for object detector...")
        rospy.wait_for_message(self.detection_topic, BoundingBoxesDist)
        rospy.loginfo("Object Detector is ready")
        rospy.loginfo("Waiting for vehicle connection...")
        self.wait_for_FCU_connection()
        rospy.loginfo("Vehicle connected! Switch to POSCTL first to make sure vehicle is safe to fly offboard")
        self.wait_for_posctl_mode()
        rospy.loginfo("Vehicle in POSCTL. Once area is clear, arm and switch to Offboard to begin mission")

        while not rospy.is_shutdown():
            next_pos = None
            if self.flight_mode == FlightMode.PRE_TAKEOFF:
                if self.EMERGENCY:
                    rospy.logwarn("Person in close vicinity, unsafe to takeoff")
                    next_pos = None
                elif not self.current_state.mode == "OFFBOARD" and not self.current_state.armed:
                    rospy.loginfo("Waiting for offboard mode and arming")
                    # Dummy waypoint to switch to offboard
                    next_pos = np.array([0, 0, 0])
                else:
                    rospy.loginfo(f"Mission start, first position: {self.get_next_pos()}")
                    self.flight_mode = FlightMode.MISSION
                    next_pos = self.get_next_pos()

            elif self.flight_mode == FlightMode.MISSION:
                if self.count >= self.num_of_pos:
                    self.flight_mode = self.flight_mode.PRE_LANDING
                    next_pos = self.safe_position
                else:
                    next_pos = self.get_next_pos()

            elif self.flight_mode == FlightMode.PRE_LANDING:
                if self.EMERGENCY:
                    self.flight_mode = FlightMode.EMERGENCY_RETREAT
                    next_pos = self.safe_position
                elif self.position_reached(self.prelanding_pos):
                    self.flight_mode = FlightMode.LANDING
                    next_pos = None
                    self.land()
                else:
                    next_pos = self.prelanding_pos

            elif self.flight_mode == FlightMode.EMERGENCY_RETREAT:
                rospy.logwarn(f"A person is in close vicinity, retreat to safe position {self.safe_position}")
                if self.EMERGENCY:
                    next_pos = self.safe_position
                else:
                    self.flight_mode = FlightMode.PRE_LANDING
                    next_pos = self.prelanding_pos

            elif self.flight_mode == FlightMode.LANDING:
                self.land()
                if self.current_local_pos[2] <= self.landing_threshold and self.LANDED:
                    rospy.loginfo("Landed vehicle, bye!")
                    self.LANDED = True

            if next_pos is not None:
                self.publish_setpoint(next_pos[0], next_pos[1], next_pos[2])

            self.rate.sleep()  # sleeps long enough to maintain desired rate


def main():
    offboard_node = OffboardNode()
    offboard_node.fly()
