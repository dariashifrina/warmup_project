#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from math import pi

# How close we will get to wall.
distance = 0.4

class DrawASquare(object):
    """ This node walks the robot to wall and stops """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("walk_to_wall")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        self.angular_speed = 0.3
        self.relative_angle = 190 * pi / 360

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.

        #if data.ranges[0] >= distance:
            # Go forward if not close enough to wall.
        firstTime = rospy.Time.now().to_sec()
        curTime = rospy.Time.now().to_sec() - firstTime
        self.twist.linear.x = 0.1
        while(curTime < 10):
            curTime = rospy.Time.now().to_sec() - firstTime
            self.twist_pub.publish(self.twist)
        self.twist.linear.x = 0
        self.twist.angular.z = -abs(self.angular_speed)
        current_angle = 0
        firstTime = rospy.Time.now().to_sec()
        while current_angle < self.relative_angle:
            #print(current_angle)
            curTime = rospy.Time.now().to_sec() - firstTime
            current_angle = self.angular_speed*(curTime)
            self.twist_pub.publish(self.twist)
        print("turned 90")
        self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)
        rospy.Rate(10).sleep()
        # Publish msg to cmd_vel.
        #self.twist_pub.publish(self.twist)


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = DrawASquare()
    node.run()