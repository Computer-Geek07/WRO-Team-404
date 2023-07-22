#!/usr/bin/env python3

"""
This script is used to create a dataset by opening up a video file and publishing the frames to a ROS topic.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

VISUALIZE = True
VID_FIL_LOC = (
    "/home/arjun/catkin_ws/src/WRO-matrix-testing/nav_bot/src/vidSamp.mp4"
)


def dataset_creator():
    """
    This function is used to create a dataset by opening up a video file
    and publishing the frames to a ROS topic.
    """
    rospy.init_node("dataset_creator", anonymous=False)
    pub = rospy.Publisher("image", Image, queue_size=10)
    rate = rospy.Rate(30)
    bridge = CvBridge()

    # Open the video file at the specified location
    cap = cv2.VideoCapture(VID_FIL_LOC)

    while not rospy.is_shutdown():
        # Read the frame
        ret, frame = cap.read()

        try:
            # Resize the frame
            frame = cv2.resize(frame, (640, 480))
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Publish the frame
        pub.publish(image_message)

        # Visualize the frame
        if VISUALIZE:
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        rate.sleep()

    # Release the video file
    cap.release()

    # Close all the windows
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        dataset_creator()
    except rospy.ROSInterruptException:
        pass