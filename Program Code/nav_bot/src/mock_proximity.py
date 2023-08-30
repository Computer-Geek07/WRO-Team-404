#!/usr/bin/env python3

"""
Create a ROS node that publishes the Proximity_Dist message
"""

import rospy

from nav_bot.msg import Proximity_Dist

MOCK_PROX_DIST = True

def publisher():
    # Initialize the ROS node
    rospy.init_node("mock_proximity_generator", anonymous=False)

    # Create a Proximity_Dist message object
    proximity_dist = Proximity_Dist()

    # Initialize the Proximity_Dist message
    if MOCK_PROX_DIST:
        proximity_dist.prox_dist_l = 35
        proximity_dist.prox_dist_r = 35

    # Create a publisher object
    proximity_dist_pub = rospy.Publisher("proximity_dist", Proximity_Dist, queue_size=10)

    # Define the loop rate
    rate = rospy.Rate(30)

    # Create a while loop
    while not rospy.is_shutdown():
        # Publish the Proximity_Dist message
        proximity_dist_pub.publish(proximity_dist)

        # Sleep for the remainder of the loop
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
