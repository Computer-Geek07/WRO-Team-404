#!/usr/bin/env python3

"""
A ROS node that generates a trajectory for the robot to follow
It subscribes to the object_tracker_node and generates input for creating cmd_vel messages
"""

import rospy
from nav_bot.msg import BoundingBox_Params
from nav_bot.msg import Traj_Params
from nav_bot.msg import Proximity_Dist


class trajectory_generator(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("trajectory_generator_node", anonymous=False)

        # Create a variable to populate the Traj_Params message
        self.traj_params = Traj_Params()

        # Create a flag to signal the presence of the object
        self.object_present = False

        #* Initialize the Traj_Params message data

        # Forward movement
        self.traj_params.fwdSpeed = 100

        # Turning
        self.traj_params.dir = False
        self.traj_params.tSpeed = 0 # Technically intensity but I dont care

        # Proximity
        self.traj_params.r_proxy_flag = False
        self.traj_params.l_proxy_flag = False

        # Sharp turns
        self.traj_params.l_corner = False
        self.traj_params.r_corner = False



        # Create a few decision variables
        self.object_center_x = 0
        self.object_color = ""
        self.object_location = ""
        self.object_fov_area = 0

        self.robot_wall_proximity_left = 0
        self.robot_wall_proximity_right = 0
        self.robot_goto_position = ""

        # Define the steering zero position
        self.steering_zero_position = 90

        self.centerCompensation = 200

        # Hardcode the maximum image width and height
        self.max_width = 640
        self.max_height = 480

        # Divide the image into 2 halves
        self.half_width = self.max_width / 2


        #! Largest object on screen Bbox_Params
        # Initialize the bbox_params message
        self.bbox_params = BoundingBox_Params()

        # Define the maximum length of the bounding box parameters list
        self.bbox_params_list_maxlen = 10

        # Create a list to store the last 10 bounding box parameters
        self.bbox_params_list = []

        #! Second largest object on screen Bbox_Params
        # Initialize the bbox_params message
        self.bbox_params2 = BoundingBox_Params()

        # Define the maximum length of the bounding box parameters list
        self.bbox_params_list_maxlen2 = 10

        # Create a list to store the last 10 bounding box parameters
        self.bbox_params_list2 = []



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
        self.traj_params.proximity_dist_r = data.proximity_dist_r
        self.traj_params.proximity_dist_l = data.proximity_dist_l

    def bbox_params_callback(self, data):
        if data.box_type == "max":
            # Get the bounding box parameters
            self.bbox_params = data
            # Call the clean_object_identifier method
            self.clean_object_identifier()
        
        else:
            pass
            # self.bbox_params2 = data
            # self.clean_object_identifier()

    def clean_object_identifier(self):
        """
        A method to average the last 10 bounding box parameters,
        run a check if the object is present within this average,
        and then turn on a flag to signal the presence of the object
        """

        # update the latest 10 bounding box parameters in the list
        # and discard previous when newer ones are available
        if len(self.bbox_params_list) < self.bbox_params_list_maxlen:
            self.bbox_params_list.append(self.bbox_params)
            return

        # Calculate the average of the last 10 bounding box parameters
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


        

        #TODO Add correct colour reading
        self.avg_bbox_params.color = self.bbox_params.color

        # Check the bounding box parameters list for all elements, compare with the average,
        # and if the difference is less than 10%, then turn on the object_present flag
        # for i in self.bbox_params_list:
        #     if (
        #         abs(i.x - self.avg_bbox_params.x) < 0.1 * self.avg_bbox_params.x
        #         and abs(i.y - self.avg_bbox_params.y) < 0.1 * self.avg_bbox_params.y
        #         and abs(i.width - self.avg_bbox_params.width)
        #         < 0.1 * self.avg_bbox_params.width
        #         and abs(i.height - self.avg_bbox_params.height)
        #         < 0.1 * self.avg_bbox_params.height
        #     ):
        #         self.object_present = True
        #     else:
        #         self.object_present = False
        #         break

        for i in self.bbox_params_list:
            if (
                abs(i.x - self.avg_bbox_params.x) < 0.1 * self.avg_bbox_params.x    
            ):
                self.object_present = True
            else:
                self.object_present = False
                break

        # Reset the bounding box parameters list
        #TODO Look into doing this in a more efficient way
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
        self.object_color = self.avg_bbox_params.color

        # Calculate the object location whether it is at the left or right of the image
        if self.object_center_x < self.half_width:
            self.object_location = "left"
        else:
            self.object_location = "right"

        # Find the robot goto position
        # If the object is red, robot should go to the right
        # If the object is green, robot should go to the left
        if self.object_color == "red":
            self.robot_goto_position = "right"
        elif self.object_color == "green":
            self.robot_goto_position = "left"

        # # Calculate the robot wall proximity
        # self.robot_wall_proximity_left = self.traj_params.proximity_dist_l
        # self.robot_wall_proximity_right = self.traj_params.proximity_dist_r

        # Calculate the object field of view area
        self.object_fov_area = self.avg_bbox_params.width * self.avg_bbox_params.height

        # Call the generate_trajectory method
        self.generate_trajectory()

    def generate_trajectory(self):
        """
        A method to generate a trajectory for the robot to follow based on updated decision parameters
        """

        # Default thing to do is to keep going straight
        # Read 90 degrees as the center
        self.traj_params.fwdSpeed = 100
        self.traj_params.tSpeed = self.steering_zero_position

        # # If goto_position is left and object_location is left, then turn left
        # if self.robot_goto_position == "left" and self.object_location == "left":
        #     self.traj_params.fwdSpeed = 100
        #     self.traj_params.tSpeed = self.steering_zero_position - 20

        # # If goto_position is right and object_location is right then turn right
        # elif self.robot_goto_position == "right" and self.object_location == "right":
        #     self.traj_params.fwdSpeed = 100
        #     self.traj_params.tSpeed = self.steering_zero_position + 20

        # # If goto_position is left and object_location is right then go straight
        # elif self.robot_goto_position == "left" and self.object_location == "right":
        #     self.traj_params.fwdSpeed = 100
        #     self.traj_params.tSpeed = self.steering_zero_position

        # # If goto_position is right and object_location is left then go straight
        # elif self.robot_goto_position == "right" and self.object_location == "left":
        #     self.traj_params.fwdSpeed = 100
        #     self.traj_params.tSpeed = self.steering_zero_position

        # # If goto_position is left and object_location is not defined then go straight
        # elif self.object_location == "":
        #     self.traj_params.fwdSpeed = 100
        #     self.traj_params.tSpeed = self.steering_zero_position

        # # If object_fov_area is greater than object_max_allowed_fov_area then slow down 
        # # and reverse the robot
        # elif self.object_fov_area > 0.8 * self.max_width * self.max_height:
        #     self.traj_params.fwdSpeed = -100
        #     self.traj_params.tSpeed = self.steering_zero_position
        
        # If goto_position is left ensure that object is at centerDistance from the half_width of the camera footage
        print(self.robot_goto_position)
        if self.robot_goto_position == "left":
            if not self.object_center_x == self.half_width - self.centerCompensation:
                print("LEFT")
                self.traj_params.fwdSpeed = 80
                self.traj_params.tSpeed = self.steering_zero_position - 45

        # If goto_position is right ensure that object is at centerDistance from the half_width of the camera footage
        elif self.robot_goto_position == "right":
            if not self.object_center_x == self.half_width + self.centerCompensation:
                print("RIGHT")
                self.traj_params.fwdSpeed = 80
                self.traj_params.tSpeed = self.steering_zero_position + 45

        # Publish the Traj_Params message with the updated values
        # self.traj_params_pub.publish(self.traj_params)
    

        self.testingShowVariables()

        
    def testingShowVariables(self):
        print("self.traj_params.fwdSpeed: ", str(self.traj_params.fwdSpeed))
        print("self.traj_params.tSpeed: ", str(self.traj_params.tSpeed))



        
if __name__ == "__main__":
    tj = trajectory_generator()
