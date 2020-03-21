#!/usr/bin/env python

import rospy
import time
from skateboard.msg import Forces

def create_msg_forces(counter):
    front = counter % 52
    back = counter % 53
    msg = Forces()
    msg.header.frame_id = "board"
    msg.header.stamp = rospy.Time.now()
    msg.seq_num = counter
    msg.time_micro = int(round(time.time() * (10**6)))
    msg.front = front
    msg.back = back
    msg.diff = int(front-back)

    return msg

if __name__ == '__main__':
    counter_forces = 0
    rospy.init_node('skateboard_fake_talker')

    pub_forces = rospy.Publisher('/skateboard/Forces', Forces, queue_size=10)

    while not rospy.is_shutdown():
        msg = create_msg_forces(counter_forces)
        counter_forces += 1
        pub_forces.publish(msg)
        time.sleep(0.05)
