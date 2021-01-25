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
phase = 0
angle = 0
firstTime = 0
curTime = 0
current_angle = 0

class PersonFollower(object):
    """ This node tries to walk the robot to the nearest object"""

    def __init__(self):
        # Start rospy node.
        rospy.init_node("person_follower")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        self.threshold = 5
        self.angular_speed = 0.2

    def process_scan(self, data):
        '''
        Determine the closest object by scanning every degree.
        Once degree identified, turn robot to that degree.
        Go forward until what is directly ahead is too close. then stop
        scan again for nearest object and make sure its not too close to track
        '''
        global phase, angle, firstTime, curTime, current_angle
        if(phase == 0):
            angle_index = -1
            min = 999
            for i in range(len(data.ranges)):
                if data.ranges[i] < min and data.ranges[i] > distance:
                    min = data.ranges[i]
                    angle_index = i
            if(angle_index == -1):
                return           
            angle = (angle_index + 3) * pi / 180
            phase = 1
            firstTime = rospy.Time.now().to_sec()
            current_angle = 0
            self.twist.angular.z = abs(self.angular_speed)
        elif(phase == 1):
            if(current_angle < angle):
                curTime = rospy.Time.now().to_sec() - firstTime
                current_angle = self.angular_speed*(curTime)
            else:
                self.twist.angular.z = 0
                phase = 2
                self.twist.linear.x = 0.5
        elif(phase == 2):
            if data.ranges[0] > distance:
                phase = 2
            else:
                self.twist.linear.x = 0
                phase = 0
    
        self.twist_pub.publish(self.twist)


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()