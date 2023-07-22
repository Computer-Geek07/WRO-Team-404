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
from nav_bot.msg import BoundingBox_Params

# Define the color ranges for red in HSV color space
red_lower = np.array([0, 116, 88])
red_upper = np.array([179, 255, 255])

# Define the color ranges for green in HSV color space
green_lower = np.array([23, 45, 13])
green_upper = np.array([72, 195, 255])

# Minimum contour area threshold to filter out noise
min_contour_area = 1000


def detect_objects():
    # Initialize the ROS node
    rospy.init_node("object_tracker_node", anonymous=False)

    # Create a publisher object to publish the bounding box parameters
    bbox_params_pub = rospy.Publisher("bbox_params", BoundingBox_Params, queue_size=10)

    # Create a VideoCapture object
    cap = cv2.VideoCapture(0)

    # used to record the time when we processed last frame
    prev_frame_time = 0

    # used to record the time at which we processed current frame
    new_frame_time = 0

    bbox_params = BoundingBox_Params()

    global_x = 0
    global_y = 0
    global_width = 0
    global_height = 0
    global_color = ""

    while not rospy.is_shutdown():
        # Read a frame from the video capture
        ret, frame = cap.read()

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply a Gaussian blur to the frame
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Create color masks for red and green
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Combine color masks
        combined_mask = cv2.bitwise_or(red_mask, green_mask)

        # Perform Canny edge detection on the combined mask
        edges = cv2.Canny(combined_mask, threshold1=30, threshold2=100)

        # Find contours in the edges
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # contours, _ = cv2.findContours(
        #     combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        # )

        # Filter contours based on area
        filtered_contours = [
            cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area
        ]

        # Draw bounding boxes and calculate center positions for filtered contours
        for contour in filtered_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(frame, (center_x, center_y), 3, (0, 255, 0), -1)
            global_x = x
            global_y = y
            global_width = w
            global_height = h
            global_color = "green"

        # Calculate the time it took to process the frame
        new_frame_time = time.time()

        # Calculate the frame rate
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time

        # Display the frame rate on terminal
        # print("FPS: ", int(fps))

        # Display the frame
        cv2.imshow("Cuboid Box Detection", frame)
        cv2.imshow("Combined Masked", combined_mask)
        cv2.imshow("Edges", edges)

        # Populate the bounding box parameters before publishing
        # bbox_params.header.stamp = rospy.Time.now()
        # bbox_params.header.frame_id = "camera_frame"
        bbox_params.x = global_x
        bbox_params.y = global_y
        bbox_params.width = global_width
        bbox_params.height = global_height
        bbox_params.color = global_color

        # Publish the bounding box parameters
        bbox_params_pub.publish(bbox_params)

        # Sleep for 0.1 seconds
        # rospy.sleep(0.1)

        # Check for the 'q' key press to exit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Release the video capture object and close the windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        detect_objects()
    except rospy.ROSInterruptException:
        pass
