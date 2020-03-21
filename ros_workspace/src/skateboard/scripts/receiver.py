#!/usr/bin/env python
import socket
import struct
import sys
import time
import rospy
from skateboard.msg import Forces
from skateboard.msg import Gyro
from skateboard.msg import Acc
from skateboard.msg import Window

def process_datapoint(datapoint):
    id = datapoint[0]

    if id == 0: # loadcell front-back
        front = datapoint[1]
        back = datapoint[2]
        diff = datapoint[3]
        velocity = datapoint[4]

        global pub_forces

        msg = Forces()
        msg.header.frame_id = "board"
        msg.header.stamp = rospy.Time.now()
        msg.front = front
        msg.back = back
        msg.diff = diff

        pub_forces.publish(msg)
    if id == 1: # loadcell front-back
        pass
    if id == 2: # acceleration x, y, z
        x = datapoint[1]
        y = datapoint[2]
        z = datapoint[3]

        global pub_acc

        msg = Acc()
        msg.header.frame_id = "controller"
        msg.header.stamp = rospy.Time.now()
        msg.x = x
        msg.y = y
        msg.z = z

        pub_acc.publish(msg)

    if id == 3: # gyro x, y, z
        x = datapoint[1]
        y = datapoint[2]
        z = datapoint[3]

        global pub_gyro

        msg = Gyro()
        msg.header.frame_id = "controller"
        msg.header.stamp = rospy.Time.now()
        msg.x = x
        msg.y = y
        msg.z = z

        pub_gyro.publish(msg)

    if id == 4: # window midpoint, lower, upper, diff_scaled
        mid = datapoint[1]
        lower = datapoint[2]
        upper = datapoint[3]
        diff_scaled = datapoint[4]

        global pub_window

        msg = Window()
        msg.header.frame_id = "controller"
        msg.header.stamp = rospy.Time.now()
        msg.mid = mid
        msg.lower = lower
        msg.upper = upper
        msg.diff_scaled = diff_scaled

        pub_window.publish(msg)

def recv_datapoints():
    data, addr = recv_sock.recvfrom(sdp_single_sizeof * max_per_datagram)
    num_datapoints = len(data) / sdp_single_sizeof

    datapoints = []

    if  num_datapoints > 0:

        for i in range(num_datapoints):
            id = struct.unpack_from("B", data, i * sdp_single_sizeof)[0]


            if id in [0, 1, 4]:
                datapoints.append(struct.unpack_from(sdp_single_format_integer,
                                                     data, i * sdp_single_sizeof))
            else:
                datapoints.append(struct.unpack_from(sdp_single_format_float,
                                                     data, i * sdp_single_sizeof))
    return datapoints

sdp_single_format_integer = "Biiii"
sdp_single_format_float = "Bffff"
sdp_single_sizeof = struct.calcsize(sdp_single_format_integer)
sdp_single_field_count = len(sdp_single_format_integer)


max_per_datagram = 32

own_ip = "192.168.0.2"
recv_port = 6354
recv_sock = socket.socket(socket.AF_INET,    # Internet
                          socket.SOCK_DGRAM) # UDP
recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, sdp_single_sizeof)
recv_sock.bind((own_ip, recv_port))

if __name__ == '__main__':
    rospy.init_node('skateboard_talker')
    global pub_forces
    pub_forces = rospy.Publisher('/skateboard/Forces', Forces, queue_size=10)

    global pub_gyro
    pub_gyro = rospy.Publisher('/skateboard/Gyro', Gyro, queue_size=10)

    global pub_acc
    pub_acc = rospy.Publisher('/skateboard/Acc', Acc, queue_size=10)

    global pub_window
    pub_window = rospy.Publisher('/skateboard/Window', Window, queue_size=10)

    while not rospy.is_shutdown():
        datapoints = recv_datapoints()
        if not datapoints == None:
            for i in range(len(datapoints)):
                process_datapoint(datapoints[i])
