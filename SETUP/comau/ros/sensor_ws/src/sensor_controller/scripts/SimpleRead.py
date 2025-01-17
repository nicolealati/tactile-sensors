#!/usr/bin/env python3

import serial
import struct
import numpy as np
import time
import rospy
from sensor_controller.msg import Piezosensor
from sensor_controller.srv import Tare, TareResponse

class SensorController:
    def __init__(self):
        rospy.init_node('sensor_controller', anonymous=False)

        self.Pub = rospy.Publisher('/piezosensor', Piezosensor, queue_size=10)
        self.TareService = rospy.Service('/tare', Tare, self.tare)

        self.start_tare = True
        self.tare_values = np.array([0 for i in range(40)])
        self.tare_base = np.array([0 for i in range(40)])
        self.tare_window = 1000
        self.tare_counter = 0

        self.sensors = serial.Serial('/dev/ttyUSB0', 1000000)

        self.header = "3c3e00"
        self.n_bytes = 83

        self.start_string = "6368 6E3F FFFF FF00 0003 FF0A"
        self.start_string = self.start_string.replace(" ", "")
        self.start_bytes = bytes.fromhex(self.start_string)

        self.stop_string = "6368 6E00 0000 0000 0000 000A"
        self.stop_string = self.stop_string.replace(" ", "")
        self.stop_bytes = bytes.fromhex(self.stop_string)

        self.mapping = {"thumb": [7, 3, 6, 10, 4, 5, 9, 8],
                        "index": [12, 16, 13, 2, 15, 14, 1, 11],
                        "middle": [20, 24, 21, 17, 23, 22, 18, 19],
                        "ring": [28, 32, 29, 25, 31, 30, 26, 27],
                        "little": [36, 40, 37, 33, 39, 38, 34, 35]
                        }
        self.thumb_data = [0 for i in range(8)]
        self.index_data = [0 for i in range(8)]
        self.middle_data = [0 for i in range(8)]
        self.ring_data = [0 for i in range(8)]
        self.little_data = [0 for i in range(8)]
        self.rate = rospy.Rate(330)

    def start(self):
        self.sensors.write(self.start_bytes)
        rospy.loginfo("Sensor controller started")

    def stop(self):
        self.sensors.write(self.stop_bytes)
        rospy.loginfo("Sensor controller stopped")
    
    def read(self):
        data = self.sensors.read(83)
        data = data.hex()
        return data
    
    def find_header(self):
        header = self.sensors.read(len(self.header)//2)
        header = header.hex()
        while header != self.header:
            addheader = self.sensors.read(1)
            addheader = addheader.hex()
            header = header[2:] + addheader
        self.sensors.read(self.n_bytes - len(self.header)//2)

    def extract_bytes(self, data, n_bytes = 2):
        n_values = 2*n_bytes
        data = data[len(self.header):]
        data = [data[i:i+n_values] for i in range(0, len(data), n_values)]
        data = [int(value, 16) for value in data]
        return data

    def collect_data(self, data):
        for i,value in enumerate(data):
            index = i + 1
            if index in self.mapping["thumb"]:
                self.thumb_data[self.mapping["thumb"].index(index)] = value
            elif index in self.mapping["index"]:
                self.index_data[self.mapping["index"].index(index)] = value
            elif index in self.mapping["middle"]:
                self.middle_data[self.mapping["middle"].index(index)] = value
            elif index in self.mapping["ring"]:
                self.ring_data[self.mapping["ring"].index(index)] = value
            elif index in self.mapping["little"]:
                self.little_data[self.mapping["little"].index(index)] = value

    def publish_data(self):
        zero_data = [0 for i in range(8)]
        msg = Piezosensor()
        msg.thumb = self.thumb_data
        msg.index = self.index_data
        msg.middle = self.middle_data
        msg.ring = self.ring_data
        msg.little = self.little_data
        self.Pub.publish(msg)
        self.rate.sleep()

    def tare(self, req):
        self.start_tare = True
        return TareResponse(True)
    
    def check_tare(self, data):
        if self.start_tare and self.tare_counter < self.tare_window:
            self.tare_base += np.array(data)
            self.tare_counter += 1
        elif self.start_tare and self.tare_counter == self.tare_window:
            self.tare_values = self.tare_base//self.tare_window
            self.start_tare = False
            self.tare_counter = 0
            self.tare_base = np.array([0 for i in range(40)])
            rospy.loginfo("Tare completed")

    def run(self):
        self.start()
        self.find_header()
        while not rospy.is_shutdown():
            data = self.read()
            data = self.extract_bytes(data)
            correct_data = list(np.array(data) - self.tare_values)
            self.collect_data(correct_data)
            self.publish_data()
            self.check_tare(data)

        self.stop()

if __name__ == '__main__':
    sensor_controller = SensorController()
    sensor_controller.run()