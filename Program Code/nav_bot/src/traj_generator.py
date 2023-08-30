#!/usr/bin/env python3

"""
A ROS node that generates a trajectory for the robot to follow
It subscribes to the object_tracker_node and generates input for creating cmd_vel messages
"""

import rospy
from nav_bot.msg import BoundingBox_Params
from nav_bot.msg import Traj_Params
from nav_bot.msg import Proximity_Dist

FOV_AREA_FACTOR = 10
WALL_CLOSE_PROXIMITY = 0.10
WALL_CENTRE_PROXIMITY = 0.25


class trajectory_generator(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("trajectory_generator_node", anonymous=False)

        # Create a variable to populate the Traj_Params message
        self.traj_params = Traj_Params()

        # Create a flag to signal the presence of the object
        self.object_present = False

        # Initialize the bbox_params message
        self.bbox_params = BoundingBox_Params()
        self.avg_bbox_params = BoundingBox_Params()

        # Create a few decision variables
        self.object_center_x = 0
        self.object_color = ""
        self.object_location = ""
        self.robot_wall_proximity_left = 0
        self.robot_wall_proximity_right = 0
        self.object_fov_area = 0
        self.object_fov_area_percentage = 0
        self.robot_goto_position = ""

        # Hardcode the maximum image width and height
        self.max_width = 640
        self.max_height = 480

        # Divide the image into 2 halves
        self.half_width = self.max_width / 2

        # Create a list to store the last 5 bounding box parameters
        self.bbox_params_list = []
        self.bbox_params_element_size = 5

        # Define the loop rate
        self.rate = rospy.Rate(30)

        # Create a publisher object to help create cmd_vel messages
        self.traj_params_pub = rospy.Publisher(
            "traj_params", Traj_Params, queue_size=10
        )

        # Create a subscriber object to subscribe to the bbox_params topic
        rospy.Subscriber("bbox_params", BoundingBox_Params, self.bbox_params_callback)

        # Create a subscriber object to subscribe to the proximity_dist topic
        rospy.Subscriber("proximity_dist", Proximity_Dist, self.proximity_dist_callback)

        # create a rospy spinner to keep the node running
        rospy.spin()

    def proximity_dist_callback(self, data):
        # Get the proximity distance
        self.robot_wall_proximity_left = data.prox_dist_l / 100
        self.robot_wall_proximity_right = data.prox_dist_r / 100

    def bbox_params_callback(self, data):
        # Get the bounding box parameters
        if data.box_type == "max":
            self.bbox_params = data
            # Call the clean_object_identifier method
            self.clean_object_identifier()

    def clean_object_identifier(self):
        """
        A method to average the last 5 bounding box parameters,
        run a check if the object is present within this average,
        and then turn on a flag to sigal the presence of the object
        """

        # update the latest 10 bounding box parameters in the list
        # and discard previous when newer ones are available
        if len(self.bbox_params_list) < self.bbox_params_element_size:
            self.bbox_params_list.append(self.bbox_params)
            return

        # Calculate the average of the last n bounding box parameters
        self.avg_bbox_params = BoundingBox_Params()
        self.avg_bbox_params.x = sum([i.x for i in self.bbox_params_list]) / len(
            self.bbox_params_list
        )
        self.avg_bbox_params.y = sum([i.y for i in self.bbox_params_list]) / len(
            self.bbox_params_list
        )
        self.avg_bbox_params.width = sum(
            [i.width for i in self.bbox_params_list]
        ) / len(self.bbox_params_list)
        self.avg_bbox_params.height = sum(
            [i.height for i in self.bbox_params_list]
        ) / len(self.bbox_params_list)
        self.avg_bbox_params.color = self.bbox_params_list[0].color

        # Check the bounding box parameters list for only one element, compare with the average,
        # and if the difference is less than 10%, then turn on the object_present flag
        for i in self.bbox_params_list:
            if abs(i.x - self.avg_bbox_params.x) < 0.1 * self.avg_bbox_params.x:
                self.object_present = True
            else:
                self.object_present = False
                break

        # Reset the bounding box parameters list
        self.bbox_params_list = []

        # Call the update_decision_params method
        if self.object_present:
            self.update_decision_params()

    def update_decision_params(self):
        """
        A method to update the decision parameters and then call the generate_trajectory method
        """
        # Calculate the object center x coordinate
        self.object_center_x = self.avg_bbox_params.x + self.avg_bbox_params.width / 2

        # Calculate the object color
        self.object_color = self.bbox_params.color

        # Calculate whether the object is on the left or right or center of the image
        # Divide the image into 3 parts with 40% on the left, 20% in the center, and 40% on the right
        if self.object_center_x < 0.4 * self.max_width:
            self.object_location = "left"
        elif self.object_center_x > 0.6 * self.max_width:
            self.object_location = "right"
        else:
            self.object_location = "center"

        # Find the robot goto position
        # If the object is red, robot should go to the right
        # If the object is green, robot should go to the left
        if self.object_color == "red":
            self.robot_goto_position = "right"
        else:
            self.robot_goto_position = "left"

        # Calculate the object field of view area and percentage
        self.object_fov_area = self.avg_bbox_params.width * self.avg_bbox_params.height
        self.object_fov_area_percentage = (
            self.object_fov_area / (self.max_width * self.max_height)
        ) * 100

        # Call the send_trajectory_params method
        self.send_trajectory_params()
        print("Sending trajectory parameters")

    def send_trajectory_params(self):
        """
        A method to publish the trajectory parameters
        Updates the Traj_Params message and publishes it
        """

        # Create a Traj_Params message object
        self.traj_params = Traj_Params()

        # Update the Traj_Params message object
        self.traj_params.go_forward = True
        self.traj_params.go_backward = False

        if self.object_location == "left":
            self.traj_params.object_loc_in_frame = "left"
        elif self.object_location == "right":
            self.traj_params.object_loc_in_frame = "right"
        elif self.object_location == "center":
            self.traj_params.object_loc_in_frame = "center"

        if self.robot_wall_proximity_right >= WALL_CENTRE_PROXIMITY:
            self.traj_params.proximity_right_straight = True

        if self.robot_wall_proximity_left >= WALL_CENTRE_PROXIMITY:
            self.traj_params.proximity_left_straight = True

        if (
            self.robot_wall_proximity_right < WALL_CENTRE_PROXIMITY
            and self.robot_wall_proximity_right > WALL_CLOSE_PROXIMITY
        ):
            self.traj_params.proximity_right_straight = False

        if (
            self.robot_wall_proximity_left < WALL_CENTRE_PROXIMITY
            and self.robot_wall_proximity_left > WALL_CLOSE_PROXIMITY
        ):
            self.traj_params.proximity_left_straight = False

        if self.robot_wall_proximity_right < WALL_CLOSE_PROXIMITY:
            self.traj_params.proximity_right_extreme = True
        else:
            self.traj_params.proximity_right_extreme = False

        if self.robot_wall_proximity_left < WALL_CLOSE_PROXIMITY:
            self.traj_params.proximity_left_extreme = True
        else:
            self.traj_params.proximity_left_extreme = False

        if self.object_fov_area_percentage < FOV_AREA_FACTOR:
            self.traj_params.safe_fov_area = True

        if self.object_fov_area_percentage >= FOV_AREA_FACTOR:
            self.traj_params.safe_fov_area = False

        if self.robot_goto_position == "left":
            self.traj_params.object_turn_in_frame = "left"

        if self.robot_goto_position == "right":
            self.traj_params.object_turn_in_frame = "right"

        # Publish the Traj_Params message object
        self.traj_params_pub.publish(self.traj_params)


if __name__ == "__main__":
    tj = trajectory_generator()
