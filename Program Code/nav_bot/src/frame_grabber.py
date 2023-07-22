#!/usr/bin/env python3
"""
A Simple OpenCV2 based frame grabber that publishes the frames to a ROS topic
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def frame_grabber():
    # Initialize the ROS node
    rospy.init_node("frame_grabber_node", anonymous=False)

    # Create a publisher object to publish the image message
    image_pub = rospy.Publisher("image", Image, queue_size=10)

    # Display a log message
    rospy.loginfo("Starting frame grabber node")

    # Create a CvBridge object
    bridge = CvBridge()

    # Create a VideoCapture object
    cap = cv2.VideoCapture(0)

    # Set the rate at which to publish the image message
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to a ROS image message
        try:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Publish the image message
        image_pub.publish(image_msg)

        # Sleep for the remaining time until 10 Hz is reached
        rate.sleep()


if __name__ == "__main__":
    try:
        frame_grabber()
    except rospy.ROSInterruptException:
        pass
