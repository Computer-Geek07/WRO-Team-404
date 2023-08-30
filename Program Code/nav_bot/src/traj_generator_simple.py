#!/usr/bin/env python3

"""
A ROS node that generates a trajectory for the robot to follow
It subscribes to the object_tracker_node and generates input for creating cmd_vel messages
"""

import rospy
from nav_bot.msg import Color_Params
from nav_bot.msg import Traj_Params
from nav_bot.msg import Proximity_Dist


class trajectory_generator(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("trajectory_generator_node", anonymous=False)

        # Init global variables
        self.left_dist = 0  # meter
        self.right_dist = 0  # meter
        self.blue_color = 0
        self.green_color = 0
        self.white_color = 0

        # Global Variables for Wall Following
        self.extreme_right = 0.25
        self.extreme_left = 0.25
        self.centring_both = 0.30
        self.turn_distance = 1.26

        # Create a variable to populate the Traj_Params message
        self.traj_params = Traj_Params()

        # Define the loop rate
        self.rate = rospy.Rate(10)

        # Create a publisher object to help create cmd_vel messages
        self.traj_params_pub = rospy.Publisher(
            "traj_params", Traj_Params, queue_size=10
        )

        # Create a subscriber object to subscribe to the proximity_dist topic
        rospy.Subscriber("proxi_data", Proximity_Dist, self.proximity_dist_callback)
        rospy.Subscriber("color_params", Color_Params, self.color_params_callback)

    def proximity_dist_callback(self, msg):
        # Store the values in global variables
        self.left_dist = msg.prox_dist_l / 100
        self.right_dist = msg.prox_dist_r / 100

    def color_params_callback(self, msg):
        # Store the values in global variables
        self.blue_color = msg.blue_color
        self.green_color = msg.green_color
        self.white_color = msg.white_color

    def calc_traj_params(self):
        self.traj_params = Traj_Params()

        self.traj_params.go_forward = True
        # Check for the proxi walls and keeo the robot in centre
        if self.left_dist <= self.extreme_left:
            self.traj_params.proximity_left_extreme = True
            self.traj_params.proximity_right_extreme = False
            self.traj_params.proximity_left_straight = False
            self.traj_params.proximity_right_straight = False
            self.traj_params.object_turn_in_frame = "nothing"
            print("Left extreme")
            return

        elif self.right_dist <= self.extreme_right:
            self.traj_params.proximity_right_extreme = True
            self.traj_params.proximity_left_extreme = False
            self.traj_params.proximity_left_straight = False
            self.traj_params.proximity_right_straight = False
            self.traj_params.object_turn_in_frame = "nothing"
            print("Right extreme")
            return

        # Condition to check if its far from walls and near the centre
        elif (
            self.left_dist >= self.centring_both and self.right_dist >= self.centring_both 
            and self.left_dist <= self.turn_distance and self.right_dist <= self.turn_distance
        ):
            self.traj_params.proximity_left_extreme = False
            self.traj_params.proximity_right_extreme = False
            self.traj_params.proximity_left_straight = True
            self.traj_params.proximity_right_straight = True
            self.traj_params.object_turn_in_frame = "nothing"
            print("Going Straight")
            return

        # Check if blue color is detected
        elif self.left_dist >= self.turn_distance and self.blue_color == True:
            self.traj_params.object_turn_in_frame = "left"
            self.traj_params.go_forward = False
            print("Blue color detected")
            print("Taking a left turn")
            rospy.sleep(3)
            return

        # Check if green color is detected
        elif self.right_dist >= self.turn_distance and self.green_color == True:
            self.traj_params.object_turn_in_frame = "right"
            self.traj_params.go_forward = False
            print("Orange color detected")
            print("Taking a right turn")
            rospy.sleep(3)
            return

        # # Check if white color is detected
        # elif self.white_color == True and (
        #     self.left_dist >= self.turn_distance
        #     or self.right_dist >= self.turn_distance
        # ):
        #     self.traj_params.object_turn_in_frame = "turning"
        #     self.traj_params.go_forward = False
        #     print("White color detected")
        #     print("Waiting to turn")
        #     return

    def run(self):
        while not rospy.is_shutdown():
            self.calc_traj_params()
            # Publish the Traj_Params message
            self.traj_params_pub.publish(self.traj_params)
            print(self.traj_params)
            print("")
            self.rate.sleep()


if __name__ == "__main__":
    tj = trajectory_generator()
    tj.run()
