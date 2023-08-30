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
        rospy.init_node("har_code_vel_generator_node", anonymous=False)

        # Create a publisher object to publish the cmd_vel message
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Create a Twist message object
        self.cmd_vel = Twist()

        # Set the linear and angular velocities
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)

        self.HARD_TURNED = False

        # Call the function to generate the cmd_vel message
        self.left_until_fov()
        self.stop_robot(5.0)

        self.right_until_fov()
        self.stop_robot(5.0)

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

    def stop_robot(self, time_delay):
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)
        print("Stopping the robot for ", time_delay, " seconds")
        rospy.sleep(time_delay)


if __name__ == "__main__":
    try:
        cmd_gen = Cmd_Vel_Gen()
    except rospy.ROSInterruptException:
        pass
