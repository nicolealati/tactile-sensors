#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64
import numpy as np

rospy.init_node('signal_plotter')
pub = rospy.Publisher('/send_signal', Float64, queue_size=10)
#i want to load the force array from "/home/comau/ros/sensor_ws/src/sensor_controller/scripts/signals/trapezoidal_signal.csv"
rate = rospy.Rate(250)
signal = np.loadtxt("/home/comau/ros/sensor_ws/src/sensor_controller/scripts/signals/trapezoidal_signal.csv", delimiter=",")
rospy.loginfo("Signal loaded")
while not rospy.is_shutdown():
    input('Press enter to start the stream')
    for i in range(len(signal)):
        pub.publish(signal[i])
        rate.sleep()
