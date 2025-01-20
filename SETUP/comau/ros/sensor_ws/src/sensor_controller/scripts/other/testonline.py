#!/usr/bin/env python3

import os
import numpy as np
import rospy
from sensor_controller.msg import Piezosensor
from sensor_controller.srv import Tare, TareRequest, Tared, TaredResponse
from sensor_controller.srv import Record, RecordResponse
from scipy.signal import butter, lfilter, lfilter_zi

class RecorderSensor:
    def __init__(self):
        self.start_recording = False
        self.records_directory = "/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records"
        rospy.init_node('recorder_controller')
        # Service clients and publisher
        self.TareClient = rospy.ServiceProxy('/tare', Tare)
        rospy.wait_for_service('/tare')

        # Subscriber for the sensor data
        self.Sub = rospy.Subscriber('/piezosensor', Piezosensor, self.recorder_sensor_callback)

        # Publisher for the filtered data
        self.Pub = rospy.Publisher('/piezosensor_filtered', Piezosensor, queue_size=10)

        # Service for recording control
        self.RecordService = rospy.Service('/record', Record, self.record)

        self.TaredService = rospy.Service('/tared', Tared, self.tared)

        # Initialize storage for raw and filtered data for each finger
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

        self.filtered_thumb = []
        self.filtered_index = []
        self.filtered_middle = []
        self.filtered_ring = []
        self.filtered_little = []

        # Filter parameters
        self.Fs = 250  # Sampling frequency
        self.lowcut = 0.03
        self.highcut = 0.5
        self.order = 2
        self.b, self.a = self.bandpass_filter_init(self.Fs, self.lowcut, self.highcut, self.order)

        # Initialize filter states for each taxel
        self.zi_thumb = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_index = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_middle = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_ring = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_little = [lfilter_zi(self.b, self.a) for _ in range(8)]

    # Bandpass filter design
    def bandpass_filter_init(self, Fs, lowcut, highcut, order=2):
        nyquist = 0.5 * Fs
        low = lowcut / nyquist
        high = highcut / nyquist
        b, a = butter(order, [low, high], btype='bandpass')
        return b, a

    # Callback for sensor data
    def recorder_sensor_callback(self, data):

        # Filter the data for each taxel (for each finger)
        filtered_thumb, self.zi_thumb = self.filter_finger(data.thumb, self.zi_thumb)
        filtered_index, self.zi_index = self.filter_finger(data.index, self.zi_index)
        filtered_middle, self.zi_middle = self.filter_finger(data.middle, self.zi_middle)
        filtered_ring, self.zi_ring = self.filter_finger(data.ring, self.zi_ring)
        filtered_little, self.zi_little = self.filter_finger(data.little, self.zi_little)
        
        # Publish the filtered data to /piezosensor_filtered
        filtered_msg = Piezosensor()
        filtered_msg.thumb = [value for value in filtered_thumb]
        filtered_msg.index = [value for value in filtered_index]
        filtered_msg.middle = [value for value in filtered_middle]
        filtered_msg.ring = [value for value in filtered_ring]
        filtered_msg.little = [value for value in filtered_little]
        self.Pub.publish(filtered_msg)

        if self.start_recording:
            # Append raw data
            self.data_thumb.append(data.thumb)
            self.data_index.append(data.index)
            self.data_middle.append(data.middle)
            self.data_ring.append(data.ring)
            self.data_little.append(data.little)

            # Append filtered data
            self.filtered_thumb.append(filtered_thumb)
            self.filtered_index.append(filtered_index)
            self.filtered_middle.append(filtered_middle)
            self.filtered_ring.append(filtered_ring)
            self.filtered_little.append(filtered_little)

    # Filtering function for each finger
    def filter_finger(self, finger_data, zi_finger):
        filtered_finger = []
        for taxel in range(8):
            filtered, zi_finger[taxel] = lfilter(self.b, self.a, [finger_data[taxel]], zi=zi_finger[taxel])
            filtered_finger.append(filtered[0])
        return filtered_finger, zi_finger

    def record(self, req):
        if req.start:
            self.start_recording = False
            self.TareClient(TareRequest(True))
            self.data_thumb = []
            self.data_index = []
            self.data_middle = []
            self.data_ring = []
            self.data_little = []

            self.filtered_thumb = []
            self.filtered_index = []
            self.filtered_middle = []
            self.filtered_ring = []
            self.filtered_little = []

            rospy.sleep(5)
            self.start_recording = True
            print("Recording started")
        else:
            self.start_recording = False
            self.write_data()
            print("Recording stopped")
        return RecordResponse(True)

    # Write to npy files
    def write_data(self):
        # Create a new subfolder (incremental numbering)
        i = 0
        while True:
            if not os.path.exists(self.records_directory + "/" + str(i)):
                os.makedirs(self.records_directory + "/" + str(i))
                break
            i += 1

        # Save raw data
        np.save(self.records_directory + "/" + str(i) + "/thumb.npy", self.data_thumb)
        np.save(self.records_directory + "/" + str(i) + "/index.npy", self.data_index)
        np.save(self.records_directory + "/" + str(i) + "/middle.npy", self.data_middle)
        np.save(self.records_directory + "/" + str(i) + "/ring.npy", self.data_ring)
        np.save(self.records_directory + "/" + str(i) + "/little.npy", self.data_little)

        # Save filtered data
        np.save(self.records_directory + "/" + str(i) + "/filtered_thumb.npy", self.filtered_thumb)
        np.save(self.records_directory + "/" + str(i) + "/filtered_index.npy", self.filtered_index)
        np.save(self.records_directory + "/" + str(i) + "/filtered_middle.npy", self.filtered_middle)
        np.save(self.records_directory + "/" + str(i) + "/filtered_ring.npy", self.filtered_ring)
        np.save(self.records_directory + "/" + str(i) + "/filtered_little.npy", self.filtered_little)

        print("Data saved")
    
    def tared(self, req):
        self.zi_thumb = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_index = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_middle = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_ring = [lfilter_zi(self.b, self.a) for _ in range(8)]
        self.zi_little = [lfilter_zi(self.b, self.a) for _ in range(8)]
        rospy.loginfo("Filter states reset")
        return TaredResponse(True)

if __name__ == '__main__':
    recorder_sensor = RecorderSensor()      
    rospy.spin()
