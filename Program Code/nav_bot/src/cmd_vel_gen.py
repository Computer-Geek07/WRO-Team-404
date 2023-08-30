#! /usr/bin/env python3

"""
A Simple cmd_vel generator that publishes the cmd_vel message to a ROS topic
Takes input from a node and publishes it to the cmd_vel topic
"""

import rospy
from geometry_msgs.msg import Twist
from nav_bot.msg import Traj_Params

LIN_VEL_SCALING_FACTOR = 0.25
TURN_DELAY = 1.0
STRAIGHT_DELAY = 1.0


class Cmd_Vel_Gen(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("cmd_vel_generator_node", anonymous=False)

        # Create a publisher object to publish the cmd_vel message
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Create a Twist message object
        self.cmd_vel = Twist()

        # Set the linear and angular velocities
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        # Publish the cmd_vel message
        self.cmd_vel_pub.publish(self.cmd_vel)

        self.HARD_TURNED = False

        # Setup a subscriber for the Traj_Params message
        rospy.Subscriber("traj_params", Traj_Params, self.traj_callback)

        rospy.spin()

    def traj_callback(self, data):
        # Initialzie the cmd_vel message
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        if data.go_forward == True and data.safe_fov_area == True:
            print("Moving Forward | safe FOV | trying to go near the object")
            self.go_towards_object(data)
            self.HARD_TURNED = False
            print("HARD_TURNED reset to False")

        if data.go_forward == True and data.safe_fov_area == False:
            print("Taking a HARD TRAJ|       | trying to get away from the object")
            self.get_away_from_object(data)

        # Publish the cmd_vel message
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("Published the cmd_vel message")

    def go_towards_object(self, data):
        self.cmd_vel.linear.x = LIN_VEL_SCALING_FACTOR * 1.0

        if (
            data.proximity_right_straight == True
            and data.proximity_left_straight == True
        ):
            self.cmd_vel.angular.z = 0.0
            print("Within center distance of proxi threshold")

        if (
            data.proximity_right_straight == True
            and data.proximity_left_straight == False
        ):
            self.cmd_vel.angular.z = 0.5
            print("Taking a Left Turn for proxi zone outside")

        if (
            data.proximity_right_straight == False
            and data.proximity_left_straight == True
        ):
            self.cmd_vel.angular.z = -0.5
            print("Taking a Right Turn for proxi zone outside")

        if data.proximity_right_extreme == True:
            self.cmd_vel.angular.z = 1.0
            print("Taking an Extreme Left Turn as proxi very close")

        if data.proximity_left_extreme == True:
            self.cmd_vel.angular.z = -1.0
            print("Taking an Extreme Right Turn as proxi very close")

        # Section to make it move towards the object
        if data.object_loc_in_frame == "left":
            self.cmd_vel.angular.z += 0.2
            print("Adding a slight left turn")

        if data.object_loc_in_frame == "right":
            self.cmd_vel.angular.z -= 0.2
            print("Adding a slight right turn")

        if data.object_loc_in_frame == "center":
            self.cmd_vel.angular.z += 0.0
            print("Adding 0 togoto object")

        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        print(" ")

    def get_away_from_object(self, data):
        self.cmd_vel.linear.x = LIN_VEL_SCALING_FACTOR * 0.5

        if data.object_turn_in_frame == "left" and self.HARD_TURNED == False:
            print("Taking a Left Turn as requested after FOV")
            self.left_until_fov()
            # self.cmd_vel.angular.z = 2.0

        if data.object_turn_in_frame == "right" and self.HARD_TURNED == False:
            print("Taking a Right Turn as requested after FOV")
            self.right_until_fov()
            # self.cmd_vel.angular.z = -2.0

        # REMOVE WALL CHECK
        # # Adding in a wall check for extreme closeness
        # if data.proximity_right_extreme == True:
        #     self.cmd_vel.angular.z += 0.5
        #     print("Taking an Extreme Left Turn as proxi very close")

        # if data.proximity_left_extreme == True:
        #     self.cmd_vel.angular.z -= 0.5
        #     print("Taking an Extreme Right Turn as proxi very close")

        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        print(" ")

    def left_until_fov(self):
        # harcode a set of left turn, move straight
        self.HARD_TURNED = True
        self.cmd_vel.angular.z = 1.0
        self.cmd_vel.linear.x = 0.125 * LIN_VEL_SCALING_FACTOR
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        rospy.sleep(TURN_DELAY)

        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = 0.25 * LIN_VEL_SCALING_FACTOR
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        rospy.sleep(STRAIGHT_DELAY)

        print("Outside left turn function")

    def right_until_fov(self):
        # harcode a set of right turn, move straight
        self.HARD_TURNED = True
        self.cmd_vel.angular.z = -1.0
        self.cmd_vel.linear.x = 0.125 * LIN_VEL_SCALING_FACTOR
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        rospy.sleep(TURN_DELAY)

        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = 0.25 * LIN_VEL_SCALING_FACTOR
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        rospy.sleep(STRAIGHT_DELAY)

        print("Outside right turn function")


if __name__ == "__main__":
    try:
        cmd_gen = Cmd_Vel_Gen()
    except rospy.ROSInterruptException:
        pass
