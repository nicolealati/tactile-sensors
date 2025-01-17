#!/usr/bin/env python3

import serial
import struct
import numpy as np
import rospy
from sensor_controller.msg import Piezosensor
from sensor_controller.srv import Integral, IntegralResponse
from sensor_controller.srv import Tare, TareRequest

class IntegralSensor:
    def __init__(self):
        self.start_integral = False
        rospy.init_node('integral_controller')
        self.IntegralService = rospy.Service('/integral', Integral, self.integral)
        self.TareClient = rospy.ServiceProxy('/tare', Tare)
        rospy.wait_for_service('/tare')
        self.Sub = rospy.Subscriber('/piezosensor', Piezosensor, self.integral_sensor_callback)
        self.Pub = rospy.Publisher('/integral_sensor', Piezosensor, queue_size=10)
        self.values_thumb = [0 for i in range(8)]
        self.values_index = [0 for i in range(8)]
        self.values_middle = [0 for i in range(8)]
        self.values_ring = [0 for i in range(8)]
        self.values_little = [0 for i in range(8)]

    def integral_sensor_callback(self, data):
        if self.start_integral:
            self.values_thumb = np.add(self.values_thumb, data.thumb)
            self.values_index = np.add(self.values_index, data.index)
            self.values_middle = np.add(self.values_middle, data.middle)
            self.values_ring = np.add(self.values_ring, data.ring)
            self.values_little = np.add(self.values_little, data.little)
            msg_new = Piezosensor()
            msg_new.thumb = self.values_thumb
            msg_new.index = self.values_index
            msg_new.middle = self.values_middle
            msg_new.ring = self.values_ring
            msg_new.little = self.values_little
            self.Pub.publish(msg_new)
        

    def integral(self, req):
        if req.start:
            self.start_integral = False
            self.TareClient(TareRequest(True))
            self.values_thumb = [0 for i in range(8)]
            self.values_index = [0 for i in range(8)]
            self.values_middle = [0 for i in range(8)]
            self.values_ring = [0 for i in range(8)]
            self.values_little = [0 for i in range(8)]
            rospy.sleep(5)
            self.start_integral = True
            print("Integral started")
        else:
            self.start_integral = False
            print("Integral stopped")
        return IntegralResponse(True)

if __name__ == '__main__':
    integral_sensor = IntegralSensor()      
    rospy.spin()