#!/usr/bin/env python3

"""
Create a ROS node that subscribes the Proximity_Dist message and filters out the noise
"""

import rospy

from nav_bot.msg import Proximity_Dist


class filter_proxi(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("filter_proxi_node", anonymous=False)

        # Create a Proximity_Dist message object
        self.proximity_dist = Proximity_Dist()

        # Create 2 arrays to store the last 10 values of the proximity sensor
        self.prox_dist_l_arr = []
        self.prox_dist_r_arr = []

        self.left_array_element = 5
        self.right_array_element = 5

        # Create a publisher object
        self.proximity_dist_pub = rospy.Publisher(
            "proxi_data", Proximity_Dist, queue_size=10
        )

        # Create a subscriber object
        rospy.Subscriber(
            "/proximity_data", Proximity_Dist, self.proximity_dist_callback
        )
        rospy.spin()

    def proximity_dist_callback(self, msg):
        # Store the values in global variables
        self.proximity_dist.prox_dist_l = msg.prox_dist_l
        self.proximity_dist.prox_dist_r = msg.prox_dist_r

        # Append the values to the arrays
        self.prox_dist_l_arr.append(self.proximity_dist.prox_dist_l)
        self.prox_dist_r_arr.append(self.proximity_dist.prox_dist_r)

        # If the length of the array is less than 10, then return
        if len(self.prox_dist_l_arr) < self.left_array_element:
            return

        # Reset the entire array if the length of the array is greater than 10
        if len(self.prox_dist_l_arr) > self.left_array_element:
            self.prox_dist_l_arr = []
            self.prox_dist_r_arr = []
            return

        # If the length of the array is equal to 10, then calculate the average
        if len(self.prox_dist_l_arr) == self.left_array_element:
            avg_val_l = sum(self.prox_dist_l_arr) / self.left_array_element
            avg_val_r = sum(self.prox_dist_r_arr) / self.right_array_element

            # Check if the average values are within +-5% of all the array values
            # if yes, then update the Proximity_Dist message
            # if no, then reset the array
            for i in range(0, self.left_array_element):
                if (
                    self.prox_dist_l_arr[i] < (avg_val_l * 0.95)
                    or self.prox_dist_l_arr[i] > (avg_val_l * 1.05)
                    or self.prox_dist_r_arr[i] < (avg_val_r * 0.95)
                    or self.prox_dist_r_arr[i] > (avg_val_r * 1.05)
                ):
                    self.prox_dist_l_arr = []
                    self.prox_dist_r_arr = []
                    return

            self.proximity_dist.prox_dist_l = int(avg_val_l)
            self.proximity_dist.prox_dist_r = int(avg_val_r)

            # Publish the Proximity_Dist message
            self.proximity_dist_pub.publish(self.proximity_dist)
            print("Published the Proximity_Dist message")
            print(self.proximity_dist)
            print(" ")


if __name__ == "__main__":
    try:
        filter_proxi()
    except rospy.ROSInterruptException:
        pass
