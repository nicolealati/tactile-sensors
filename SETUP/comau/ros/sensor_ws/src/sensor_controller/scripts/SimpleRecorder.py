#!/usr/bin/env python3

import os
import numpy as np
import rospy
from sensor_controller.msg import Piezosensor
from geometry_msgs.msg import Twist
from sensor_controller.srv import Tare, TareRequest
from sensor_controller.srv import Record, RecordResponse
from std_msgs.msg import Float64

class RecorderSensor:
    def __init__(self):
        self.start_recording = False
        self.records_directory = "/home/comau/ros/sensor_ws/src/sensor_controller/scripts/records/pressure"
        rospy.init_node('recorder_controller')
        self.TareClient = rospy.ServiceProxy('/tare', Tare)
        self.Sub = rospy.Subscriber('/piezosensor', Piezosensor, self.recorder_sensor_callback)
        self.Sub2 = rospy.Subscriber('/atift_sensor/data', Twist, self.recorder_sensor_callback2)
        self.RecordService = rospy.Service('/record', Record, self.record)
        self.Pub = rospy.Publisher('/force_z', Float64, queue_size=10)
        self.values_thumb = [0 for i in range(8)]
        self.values_index = [0 for i in range(8)]
        self.values_middle = [0 for i in range(8)]
        self.values_ring = [0 for i in range(8)]
        self.values_little = [0 for i in range(8)]
        self.data_thumb = []
        self.data_index = []
        self.data_middle = []
        self.data_ring = []
        self.data_little = []
        self.sensor_values = []
        self.sensor_norm = []
        self.online_z = 0
    
    def recorder_sensor_callback(self, data):
        if self.start_recording:
            self.data_thumb.append(data.thumb)
            self.data_index.append(data.index)
            self.data_middle.append(data.middle)
            self.data_ring.append(data.ring)
            self.data_little.append(data.little)
        
    def recorder_sensor_callback2(self, data):
        self.online_z = -data.linear.z
        if self.start_recording:
            self.sensor_values.append([data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z])
            self.sensor_norm.append(np.linalg.norm([data.linear.x, data.linear.y, data.linear.z]))

            
    def publish(self):
    	self.Pub.publish(self.online_z)

    def record(self, req):
        if req.start:
            self.start_recording = False
            self.TareClient(TareRequest(True))
            self.data_thumb = []
            self.data_index = []
            self.data_middle = []
            self.data_ring = []
            self.data_little = []
            self.sensor_values = []
            self.sensor_norm = []
            rospy.sleep(5)
            self.start_recording = True
            print("Recording started")
        else:
            self.start_recording = False
            self.write_data()
            print("Recording stopped")
        return RecordResponse(True)
    
    #write to a npy file
    def write_data(self):
        #i want to create a subfolder 0 or 1 if 0 is already created and so on
        i = 0
        while True:
            if not os.path.exists(self.records_directory + "/" + str(i)):
                os.makedirs(self.records_directory + "/" + str(i))
                break
            i += 1
        np.save(self.records_directory + "/" + str(i) + "/thumb.npy", self.data_thumb)
        np.save(self.records_directory + "/" + str(i) + "/index.npy", self.data_index)
        np.save(self.records_directory + "/" + str(i) + "/middle.npy", self.data_middle)
        np.save(self.records_directory + "/" + str(i) + "/ring.npy", self.data_ring)
        np.save(self.records_directory + "/" + str(i) + "/little.npy", self.data_little)
        np.save(self.records_directory + "/" + str(i) + "/sensor_values.npy", self.sensor_values)
        np.save(self.records_directory + "/" + str(i) + "/sensor_norm.npy", self.sensor_norm)

        print("Data saved")

if __name__ == '__main__':
    recorder_sensor = RecorderSensor()      
    while not rospy.is_shutdown():
        recorder_sensor.publish()
