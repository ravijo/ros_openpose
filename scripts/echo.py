#!/usr/bin/env python
# -*- coding: utf-8 -*-

# echo.py: sample script to print ros message to terminal
# Author: Ravi Joshi
# Date: 2020/03/02

# import modules
import rospy
from ros_openpose.msg import Frame


def callback(msg):
    text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    rospy.loginfo('%s\n' % text)


def main():
    rospy.init_node('echo', anonymous=False)

    # read the parameter from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber(frame_topic, Frame, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
