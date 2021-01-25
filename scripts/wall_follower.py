#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy
from math import pi
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.5
direction = 1
chosen_dist = 999
turn = 0
firstTime = 0
curTime = 0
current_angle = 0

class WallFollower(object):
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
        self.angular_speed = 0.1
        self.relative_angle = 90 * pi/ 360

        #self.direction = 0

    def process_scan(self, data):
        global turn, chosen_dist, direction, firstTime, curTime, current_angle
        #stage 0 -> keep going until you hit a wall.
        if(turn == 0):
            if data.ranges[0] >= distance:
                # Go forward if not close enough to wall.
                self.twist.linear.x = 0.1
            else:
                # Close enough to wall, stop.
                self.twist.linear.x = 0
                turn = 1
        #stage 1 -> figure out which way to turn. set velocity of z
        elif(turn == 1):
            if(data.ranges[90] < data.ranges[270]):
                direction = -1
                chosen_dist = data.ranges[90]
            else:
                direction = 1
                chosen_dist = data.ranges[270]
            print(chosen_dist)
            turn = 2
            firstTime = rospy.Time.now().to_sec()
            current_angle = 0
            self.twist.angular.z = 0.2 * direction
        #stage 2 -> turn 90 degrees. transition to stage 0.
        elif(turn == 2):
            if(current_angle < self.relative_angle):
                curTime = rospy.Time.now().to_sec() - firstTime
                current_angle = self.angular_speed*(curTime)
            else:
                self.twist.angular.z = 0
                turn = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)


    def run(self):
        # Keep the program alive.
        rospy.spin()


if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()