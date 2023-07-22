#! /usr/bin/env python3

"""
A Simple cmd_vel generator that publishes the cmd_vel message to a ROS topic
Takes input from a node and publishes it to the cmd_vel topic
"""

import rospy
from geometry_msgs.msg import Twist


class cmd_vel_gen:
    def traj_callback(self, msg):
        """
        Callback function for the traj_params subscriber
        """
        # Set the linear and angular velocities
        # Remap msg.fwdSpeed to the range [0, 1.77] from 0 to 100
        msg.fwdSpeed = msg.fwdSpeed * 1.77 / 100
        self.cmd_vel.linear.x = msg.fwdSpeed

        # Convert msg.tSpeed from degrees per second to radians per second
        msg.tSpeed = msg.tSpeed * 3.14 / 180
        self.cmd_vel.angular.z = msg.angular.z


    def cmd_vel_generator(self):

        # Initialize the ROS node
        rospy.init_node("cmd_vel_generator_node", anonymous=False)

        # Create a publisher object to publish the cmd_vel message
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Create a Twist message object
        self.cmd_vel = Twist()

        # Subscribe to the traj_params message

        # Set the linear and angular velocities
        self.cmd_vel.linear.x = 1.77 # m/s (max 1.77 m/s)
        self.cmd_vel.angular.z = 0.0

        rospy.Subscriber("traj_params", Twist, self.traj_callback)

        # Set the rate at which to publish the cmd_vel message
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Publish the cmd_vel message
            self.cmd_vel_pub.publish(self.cmd_vel)

            # Sleep for the remaining time until 10 Hz is reached
            rate.sleep()


if __name__ == "__main__":
    try:
        cmd_vel_generator_node = cmd_vel_gen()
        cmd_vel_generator_node.cmd_vel_generator()
    except rospy.ROSInterruptException:
        pass
