#!/usr/bin/env python3

"""
A ROS node that detects objects in the frame
Takes input from a Frame Grabber node, 
finds the object in the frame,
and publishes its bbox_params on a topic
"""

import time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_bot.msg import BoundingBox_Params

VISUALIZE = False
if VISUALIZE:
    DRAW_BBOX = True
    DRAW_CONT = False
else:
    DRAW_BBOX = False
    DRAW_CONT = False


class obj_tracker(object):
    def __init__(self):
        # Define the color ranges for red in HSV color space
        self.red_lower = np.array([0, 15, 30])
        self.red_upper = np.array([5, 250, 228])

        # Define the color ranges for green in HSV color space
        self.green_lower = np.array([23, 45, 13])
        self.green_upper = np.array([72, 195, 255])

        # Minimum contour area threshold to filter out noise
        self.min_contour_area = 2000

        # Initialize the ROS node
        rospy.init_node("object_tracker_node", anonymous=False)

        # Create a publisher object to publish the bounding box parameters
        self.bbox_params_pub = rospy.Publisher(
            "bbox_params", BoundingBox_Params, queue_size=10
        )

        # Creat a variable to populate the BoundingBox_Params message
        self.bbox_params = BoundingBox_Params()

        # Initialize the bbox_params message
        self.bbox_params.box_type = ""
        self.bbox_params.x = 0
        self.bbox_params.y = 0
        self.bbox_params.width = 0
        self.bbox_params.height = 0
        self.bbox_params.color = ""

        # Variables to store the frame
        self.image = None
        self.bridge = CvBridge()

        # Define the canny edge detection parameters
        self.thresh_lower = 10
        self.thresh_upper = 180
        self.l2_gradient = True

        # Define the loop rate
        self.loop_rate = rospy.Rate(30)

        # Variables to calculate the FPS
        self.prev_frame_time = 0
        self.new_time_frame = 0

        # Subscribe to the image topic to recieve the image
        rospy.Subscriber("image", Image, self.image_callback)

        # Publisher for the parameters of the bounding box
        self.bbox_params_pub = rospy.Publisher(
            "bbox_params", BoundingBox_Params, queue_size=10
        )

    def image_callback(self, data):
        """
        Callback function to update the image variable when a new image is recieved
        """
        # Convert the image to OpenCV format
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error_code:
            print(error_code)

    def publish_bbox_params(self, x, y, width, height, color, box_type):
        """
        Function to publish the bounding box parameters
        """
        # Update the bbox_params message
        self.bbox_params.box_type = box_type
        self.bbox_params.x = x
        self.bbox_params.y = y
        self.bbox_params.width = width
        self.bbox_params.height = height
        self.bbox_params.color = color

        # Publish the bbox_params message
        self.bbox_params_pub.publish(self.bbox_params)

    def show_contours(self, frame, contours):
        """
        Function to display the contours on the frame
        """
        # Draw the contours on the frame if DRAW_CONT is True
        if DRAW_CONT:
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    def show_bbox(self, frame, x, y, width, height):
        """
        Function to display the bounding box on the frame
        """
        if DRAW_BBOX:
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255), 3)

    def find_color(self, frame, x, y, width, height):
        """
        Function to find the color of the object either red or green
        Will use the hsv color space ranges declared above to find the color
        """
        # Crop the frame to the bounding box
        cropped_frame = frame[y : y + height, x : x + width]

        # Convert the cropped frame to HSV color space
        hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

        # Calculate the mean of the hue channel
        hue_mean = np.mean(hsv[:, :, 0])

        # If the mean of the hue channel is less than 20, the object is red
        if hue_mean < 20:
            return "red"
        # If the mean of the hue channel is greater than 20, the object is green
        else:
            return "green"

    def process_frames_by_canny(self):
        """
        Function to process the read frames and detect the object,
        Uses Canny edge detection
        Preprocesses the frame by converting it to HSV color space,
        applying a Gaussian blur, and then detecting the object
        """

        # Print the type of detection used
        rospy.loginfo("Using Canny edge for object detection")

        while not rospy.is_shutdown():
            # Process the frame only if the image is not None
            if self.image is not None:
                # Convert the frame to HSV color space
                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

                # Apply a Gaussian blur to the frame
                hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

                # Detect the red object in the frame
                red_mask = cv2.inRange(hsv, self.red_lower, self.red_upper)

                # Detect the green object in the frame
                green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

                # Combine color masks
                combined_mask = cv2.bitwise_or(red_mask, green_mask)

                # Perform Canny edge detection on the combined mask
                edges = cv2.Canny(
                    combined_mask,
                    threshold1=self.thresh_lower,
                    threshold2=self.thresh_upper,
                    L2gradient=self.l2_gradient,
                )

                # Find contours in the edges
                contours, _ = cv2.findContours(
                    edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                # Find the contours above the minimum area threshold
                filtered_contours = [
                    contour
                    for contour in contours
                    if cv2.contourArea(contour) > self.min_contour_area
                ]

                # If no contours are found, continue
                if len(filtered_contours) == 0:
                    continue

                # Sort the contours in descending order of area
                filtered_contours.sort(key=cv2.contourArea, reverse=True)

                # If only one contour is found, this is the largest contour
                if len(filtered_contours) == 1:
                    # Find the bounding box parameters
                    x, y, w, h = cv2.boundingRect(filtered_contours[0])

                    # Find the color of the object detected
                    color = self.find_color(self.image, x, y, w, h)

                    # Call the publish_bbox_params function
                    self.publish_bbox_params(x, y, w, h, color, "max")

                    # Display the contours
                    self.show_contours(self.image, filtered_contours)

                    # Draw the bounding box
                    self.show_bbox(self.image, x, y, w, h)

                else:
                    # Find the bounding box parameters for the largest and second largest contours
                    x1, y1, w1, h1 = cv2.boundingRect(filtered_contours[0])
                    x2, y2, w2, h2 = cv2.boundingRect(filtered_contours[1])

                    # Find the color of the object detected
                    color1 = self.find_color(self.image, x1, y1, w1, h1)
                    color2 = self.find_color(self.image, x2, y2, w2, h2)

                    # Call the publish_bbox_params function
                    self.publish_bbox_params(x1, y1, w1, h1, color1, "max")
                    self.publish_bbox_params(x2, y2, w2, h2, color2, "second_max")

                    # Display the contours
                    self.show_contours(self.image, filtered_contours)

                    # Draw the bounding box for the largest contour
                    self.show_bbox(self.image, x1, y1, w1, h1)

                # Show the frame if VISUALIZE is True
                if VISUALIZE:
                    cv2.imshow("Frame", self.image)
                    cv2.waitKey(1)

                # wait for the next frame
                self.loop_rate.sleep()

    def process_frame_by_thresholding(self):
        """
        Function to process the read frames and detect the object,
        uses simple thresholding to detect the object
        """

        # Print the type of detection used
        rospy.loginfo("Using thresholding for object detection")

        while not rospy.is_shutdown():
            # Process the frame only if the image is not None
            if self.image is not None:
                # Convert the frame to HSV color space
                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

                # Detect the red object in the frame
                red_mask = cv2.inRange(hsv, self.red_lower, self.red_upper)
                # Detect the green object in the frame
                green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

                # Combine color masks
                combined_mask = cv2.bitwise_or(red_mask, green_mask)

                # Find contours in the combined mask
                contours, _ = cv2.findContours(
                    combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                # Find the contours above the minimum area threshold
                filtered_contours = [
                    contour
                    for contour in contours
                    if cv2.contourArea(contour) > self.min_contour_area
                ]

                # If no contours are found, continue
                if len(filtered_contours) == 0:
                    continue

                # Sort the contours in descending order of area
                filtered_contours.sort(key=cv2.contourArea, reverse=True)

                # If only one contour is found, this is the largest contour
                if len(filtered_contours) == 1:
                    # Find the bounding box parameters
                    x, y, w, h = cv2.boundingRect(filtered_contours[0])

                    # Find the color of the object detected
                    color = self.find_color(self.image, x, y, w, h)

                    # Call the publish_bbox_params function
                    self.publish_bbox_params(x, y, w, h, color, "max")

                    # Display the contours
                    self.show_contours(self.image, filtered_contours)

                    # Draw the bounding box
                    self.show_bbox(self.image, x, y, w, h)

                else:
                    # Find the bounding box parameters for the largest and second largest contours
                    x1, y1, w1, h1 = cv2.boundingRect(filtered_contours[0])
                    x2, y2, w2, h2 = cv2.boundingRect(filtered_contours[1])

                    # Find the color of the object detected
                    color1 = self.find_color(self.image, x1, y1, w1, h1)
                    color2 = self.find_color(self.image, x2, y2, w2, h2)

                    # Call the publish_bbox_params function
                    self.publish_bbox_params(x1, y1, w1, h1, color1, "max")
                    self.publish_bbox_params(x2, y2, w2, h2, color2, "second_max")

                    # Display the contours
                    self.show_contours(self.image, filtered_contours)

                    # Draw the bounding box for the largest contour
                    self.show_bbox(self.image, x1, y1, w1, h1)

                # Show the frame if VISUALIZE is True
                if VISUALIZE:
                    cv2.imshow("Frame", self.image)
                    cv2.waitKey(1)

                # wait for the next frame
                self.loop_rate.sleep()


if __name__ == "__main__":
    object_tracker_node = obj_tracker()
    # object_tracker_node.process_frames_by_canny()
    object_tracker_node.process_frame_by_thresholding()
