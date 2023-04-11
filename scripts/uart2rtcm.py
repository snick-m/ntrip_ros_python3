#!/usr/bin/python3
import serial
import rospy

import pyubx2

from rtcm_msgs.msg import Message

if __name__ == "__main__":
    rospy.init_node("uart2rtcm")
    rospy.set_param("debug", rospy.get_param("~debug", 0))

    rtcm_topic = rospy.Publisher("rtcm", Message, queue_size=10)

    serial_port = rospy.get_param("~port", "/dev/ttyACM0")
    rospy.loginfo("Using serial port: " + serial_port)
    ubx = serial.Serial(serial_port, 9600, timeout=1)

    rtcm_reader = pyubx2.UBXReader(ubx, protfilter=4)

    while not rospy.is_shutdown():
        (raw_data, parsed_data) = rtcm_reader.read()

        msg = Message()
        msg.message = raw_data
        msg.header.seq += 1
        msg.header.stamp = rospy.get_rostime()

        rospy.logdebug("Published " + str(parsed_data.identity) + " Message, Seq: " + str(msg.header.seq))
        
        rtcm_topic.publish(msg)
