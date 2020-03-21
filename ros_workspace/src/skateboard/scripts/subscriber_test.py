#!/usr/bin/env python
import rospy
from skateboard.msg import Forces

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " " + str(data.header.seq) +
                  " " + str(data.diff ))

def listener():
    rospy.init_node('subscriber_test')

    rospy.Subscriber("/skateboard/Forces", Forces, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
