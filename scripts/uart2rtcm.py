#!/usr/bin/python3
import serial
import rospy

import pyubx2

from rtcm_msgs.msg import Message

if __name__ == "__main__":
    rospy.init_node("uart2rtcm")
    rospy.set_param("debug", True)

    rtcm_topic = rospy.Publisher("rtcm", Message, queue_size=10)

    serial_port = rospy.get_param("port", "/dev/ttyACM0")
    ubx = serial.Serial(serial_port, 9600, timeout=1)
    # rtk = serial.Serial("/dev/ttyACM1" if serial_port == "/dev/ttyACM0" else "/dev/ttyACM0", 9600, timeout=1)
    
    # rtk_reader = pyubx2.UBXReader(rtk, protfilter=2|4)
    rtcm_reader = pyubx2.UBXReader(ubx, protfilter=4)

    while not rospy.is_shutdown():
        msg = Message()
        (raw_data, parsed_data) = rtcm_reader.read()
        # rtk.write(raw_data)
        msg.message = raw_data
        msg.header.seq += 1
        msg.header.stamp = rospy.get_rostime()
        rtcm_topic.publish(msg)
        # (raw_data, parsed_data) = rtk_reader.read()
        # rospy.logdebug(parsed_data)
