#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

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

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        self.angular_speed = 0.2
        self.relative_angle = 180 * pi / 360

    def run(self):
        print("HERE")
        while not rospy.is_shutdown():
            firstTime = rospy.Time.now().to_sec()
            curTime = rospy.Time.now().to_sec() - firstTime
            self.twist.linear.x = 0.1
            self.twist_pub.publish(self.twist)
            while(curTime < 10):
                curTime = rospy.Time.now().to_sec() - firstTime
            self.twist.linear.x = 0
            self.twist_pub.publish(self.twist)
            self.twist.angular.z = -abs(self.angular_speed)
            self.twist_pub.publish(self.twist)
            current_angle = 0
            firstTime = rospy.Time.now().to_sec()
            while current_angle < self.relative_angle:
                curTime = rospy.Time.now().to_sec() - firstTime
                current_angle = self.angular_speed*(curTime)
            print("turned 90")
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            rospy.Rate(10).sleep()
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = DrawASquare()
    node.run()