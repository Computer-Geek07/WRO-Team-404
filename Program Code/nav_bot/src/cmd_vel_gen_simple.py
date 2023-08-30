#! /usr/bin/env python3

"""
A Simple cmd_vel generator that publishes the cmd_vel message to a ROS topic
Takes input from a node and publishes it to the cmd_vel topic
"""

import rospy
from geometry_msgs.msg import Twist
from nav_bot.msg import Traj_Params

LIN_VEL_SCALING_FACTOR = 0.22
ANG_VEL_SCALING_FACTOR = 0.80
TURN_DELAY = 2.0
STRAIGHT_DELAY = 1.0
ALIGN_DELAY = 0.2
LIN_VEL_SCALING_FACTOR_TURN = 0.50
ANG_VEL_SCALING_FACTOR_TURN = 2.00


class Cmd_Vel_Gen(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("cmd_vel_generator_node", anonymous=False)

        # Create a publisher object to publish the cmd_vel message
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Create a Twist message object
        self.cmd_vel = Twist()

        self.time_current = 0.0
        self.time_prev = rospy.get_time()

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

        if data.go_forward == True:
            self.go_forward(data)
            self.HARD_TURNED = False

        if data.go_forward == False:
            var1 = data.object_turn_in_frame

            if var1 == "left" and self.HARD_TURNED == False:
                self.hard_code_left_90()
                self.HARD_TURNED = True
                print("LEFT HARD TURN COMPLETE")
                print("HARD_TURNED reset to True")
                print(" ")

            if var1 == "right" and self.HARD_TURNED == False:
                self.hard_code_right_90()
                self.HARD_TURNED = True
                print("RIGHT HARD TURN COMPLETE")
                print("HARD_TURNED reset to True")
                print(" ")

        # Publish the cmd_vel message
        self.cmd_vel_pub.publish(self.cmd_vel)
        # print("Published the cmd_vel message")

    def go_forward(self, data):
        self.cmd_vel.linear.x = LIN_VEL_SCALING_FACTOR * 1.0

        if data.proximity_right_extreme == True:
            self.cmd_vel.angular.z = 1.0 * ANG_VEL_SCALING_FACTOR
            print("Taking soft left")

        elif data.proximity_left_extreme == True:
            self.cmd_vel.angular.z = -1.0 * ANG_VEL_SCALING_FACTOR
            print("Taking soft right")

        elif (
            data.proximity_right_extreme == False
            and data.proximity_left_extreme == False
            and data.proximity_right_straight == True
            and data.proximity_left_straight == True
        ):
            self.cmd_vel.angular.z = 0.0
            print("Moving straight")

        elif (
            data.proximity_right_extreme == False
            and data.proximity_left_extreme == False
            and data.proximity_right_straight == False
            and data.proximity_left_straight == False
        ):
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            print("Moving Stop")

        print("lin.x: ", self.cmd_vel.linear.x, "ang.z: ", self.cmd_vel.angular.z)
        print(" ")

    def hard_code_left_90(self):
        # self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        # self.cmd_vel.angular.z = 0.0
        # print("Ready for 90 degree left turn")
        # rospy.sleep(ALIGN_DELAY)

        # make sure to call function only once in 5 seconds
        self.time_current = rospy.get_time()
        if (self.time_current - self.time_prev) < 5.0:
            print("ALREADY       HARD_TURNED is True")
            print("")
            return

        self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        self.cmd_vel.angular.z = 1.0 * ANG_VEL_SCALING_FACTOR_TURN
        print("Hard coding a left turn of 90 degrees")
        rospy.sleep(TURN_DELAY)

        self.time_prev = self.time_current

        # self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        # self.cmd_vel.angular.z = 0.0
        # print("Completing the left turn")
        # rospy.sleep(STRAIGHT_DELAY)

    def hard_code_right_90(self):
        self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        self.cmd_vel.angular.z = 0.0
        print("Ready for 90 degree right turn")
        rospy.sleep(ALIGN_DELAY)

        self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        self.cmd_vel.angular.z = -1.0 * ANG_VEL_SCALING_FACTOR_TURN
        print("Hard coding a right turn of 90 degrees")
        rospy.sleep(TURN_DELAY)

        self.cmd_vel.linear.x = 1.0 * LIN_VEL_SCALING_FACTOR_TURN
        self.cmd_vel.angular.z = 0.0
        print("Completing the right turn")
        rospy.sleep(STRAIGHT_DELAY)


if __name__ == "__main__":
    try:
        cmd_gen = Cmd_Vel_Gen()
    except rospy.ROSInterruptException:
        pass
