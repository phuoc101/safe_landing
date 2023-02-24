#!/usr/bin/env python3

import rospy
import os
import sys
import traceback
from glob import glob
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class SimImagePublisherNode:
    """
    ROS node class to publish fake images for detector, used for simulator testing, will switch to camera source
    when testing with real vehicles
    """

    def __init__(self):
        rospy.init_node("sim_image_publisher")
        rospy.loginfo("Starting SimImagePublisherNode.")

        # Define ROS publisher to publish fake images for simulator
        self.image_pub = rospy.Publisher("/picam360/image_raw", Image, queue_size=10)
        # Define publishing rate
        self.rate = rospy.Rate(20)  # in Hz
        # Define image source
        self.source = rospy.get_param("~source", "./img_source/")
        if os.path.isdir(self.source):
            self.images = sorted(glob(self.source + "*.png"))
            rospy.loginfo(f"images to show: {self.images}")
        else:
            rospy.logerr(f"{self.source} is not a directory, pls put valid source")
            return
        self.im_id = 0
        self.frame_id = 0
        # Define image bridge to convert from opencv imgs to ros msgs
        self.bridge = CvBridge()

    def generate_ros_image(self):
        """Function to show images, press q to toggle between safe and unsafe image"""
        # if source is directory, publish image from it sequentially
        while not rospy.is_shutdown():
            try:
                self.frame_id += 1
                cv2_img = cv2.imread(self.images[self.im_id])
                image_message = self.bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
                image_message.header.frame_id = str(self.frame_id)
                image_message.header.stamp = rospy.Time.now()
                self.image_pub.publish(image_message)
                self.rate.sleep()
                # Press <Space> to toggle image
                cv2.imshow("Original frame", cv2_img)
                if cv2.waitKey(1) == ord(" "):
                    self.im_id = 0 if self.im_id == -1 else -1
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Exitting...")
            except Exception:
                traceback.print_exc(file=sys.stdout)


def main():
    image_publisher = SimImagePublisherNode()
    image_publisher.generate_ros_image()


if __name__ == "__main__":
    main()
