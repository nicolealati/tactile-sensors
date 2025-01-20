#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

class TestReadValues:
    def __init__(self):

        rospy.init_node('test')
        self.TestSub = rospy.Subscriber('/atift_sensor/data', Twist, self.test_callback)
        self.TestPub = rospy.Publisher('/force_z', Float64, queue_size=10)
        self.online_force = 0

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.axis(False)
        self.ax.grid(False)

    def test_callback(self, data):
        self.online_force = -data.linear.z

    def UpdateFigure(self, frame):
        self.ax.clear()
        line = Rectangle((-1, self.online_force), 2, 0.5, color='black')
        self.ax.add_patch(line)
        self.ax.set_ylim([-10, 10])
        self.ax.axis(True)
        self.ax.grid(True)
        
          
    def Run(self):
        self.TestPub.publish(self.online_force) 
        self.start_time = None
        self.ani = FuncAnimation(self.fig, self.UpdateFigure, interval=10, cache_frame_data=False)
        plt.show()
        
if __name__ == '__main__':
    test_read_values = TestReadValues()      
    while not rospy.is_shutdown():
        test_read_values.Run()
