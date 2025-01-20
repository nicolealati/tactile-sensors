#!/usr/bin/env python3

import numpy as np
import os
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.patches import Ellipse
from matplotlib.transforms import Affine2D
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

excel_file = "/home/comau/ros/sensor_ws/src/sensor_controller/scripts/pattern.xlsx"
test = "ROLLING"

class RollingInterface:
    def __init__(self, excel_file, sheet_name):
        
        self.excel_file = excel_file
        self.sheet_name = sheet_name
        self.start_time = None

       # Figures dimensions
        self.square = [2.5, 2.5]
        self.ellipse_1 = [1, 0.4]
        self.ellispe_2 = [0.8, 0.8]
        self.rect = [0.1, 0.25]
        self.line = [self.rect[0], 0.01]
        self.scale_line = 1.5
        self.line_ellipse = [self.scale_line*self.ellipse_1[0], self.scale_line*self.ellispe_2[0]]
        
        self.n_frame = 1000

        # Load patterns from excel file
        self.df_pattern = pd.read_excel(self.excel_file, sheet_name=self.sheet_name)

        self.LoadPatterns(self.df_pattern)
        self.CalculateInterpolation()

        # Subscriber e callback
        rospy.init_node('rolling_int')
        self.SubForce = rospy.Subscriber('/atift_sensor/data', Twist, self.SensorCallback)
        self.applied_force = 0
        self.scaled_force = 0

        # Create the figure
        self.fig, self.axs = plt.subplots(1, 2, figsize=(10, 5))

    def CleanTerminal(self):
        if os.name == 'nt':
            os.system('cls')

    def SensorCallback(self, data):
        self.applied_force = -data.linear.z
        self.scaled_force = (self.applied_force-0)*self.rect[1]/0.25

    def LoadPatterns(self, df_pattern): 
        self.time_step = np.array(df_pattern['time_steps'].astype(int))
        self.x_coord = np.array(df_pattern['x_position'].astype(int))
        self.y_coord = np.array(df_pattern['y_position'].astype(int))
        self.angle_rot_1 = np.array(df_pattern['rotation_angle_1'].astype(int))/180*np.pi
        self.angle_rot_2 = np.array(df_pattern['rotation_angle_2'].astype(int))/180*np.pi
        self.required_force = np.array(df_pattern['required_force'].astype(float))
        self.required_force = self.required_force[~np.isnan(self.required_force)]
        self.tolerance = np.array(df_pattern['tolerance'].astype(float))
        self.tolerance = self.tolerance[~np.isnan(self.tolerance)]
        
    def CalculateInterpolation(self):
        self.time_interp = np.linspace(0, self.time_step[-1], self.n_frame)
        self.x_interp = np.interp(self.time_interp, self.time_step, self.x_coord)
        self.y_interp = np.interp(self.time_interp, self.time_step, self.y_coord)
        self.angle_interp_1 = np.interp(self.time_interp, self.time_step, self.angle_rot_1)
        self.angle_interp_2 = np.interp(self.time_interp, self.time_step, self.angle_rot_2)
        
    def AddTransformation(self, figure, rot, ax):
        if ax is None:
            raise ValueError("Error ax.")

        transform = (Affine2D().rotate(rot) + ax.transData)
        
        figure.set_transform(transform)
        ax.add_patch(figure)

    def UpdateFigure(self, frame):
        
        for ax in self.axs:
            ax.clear()
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.axis(False)
        
        # Set titles for each subplot
        self.axs[0].set_title("FRONTAL VISION")
        self.axs[1].set_title("LATERAL VISION")

        # Initialize start_time on the first frame
        if self.start_time is None:
            rospy.loginfo("Task started")
            input('Press Enter to start the task')
            self.start_time = time.time()

        # Calculate the actual elapsed time in seconds
        elapsed_time = time.time()-self.start_time

        # Find the closest frame based on real elapsed time
        frame = np.searchsorted(self.time_interp, elapsed_time, side='right')-1

        # Stop the animation when the last frame is reached
        if frame >= len(self.time_interp)-1:
            self.ani.event_source.stop()
            plt.close()
            rospy.loginfo("Task ended")
            return

        # Moving figures
        for ax in self.axs:

            # Ground rectangle
            ax.add_patch(Rectangle((-self.square[0]/2, -self.square[1]/2), self.square[0], self.square[1], color='lightblue', alpha=0.5))

            box = Rectangle((-self.rect[0], -self.rect[1]), self.rect[0]*2, self.rect[1]*2, color='green')
            ax.add_patch(box)

            line = Rectangle((-1.5*self.line[0]-self.line[0]/2, self.scaled_force), self.line[0]*4, self.line[1], color='black')
            ax.add_patch(line)

            # Add text annotations for the axes' positions
            #ax.text(-1.5, self.y_interp[frame], f'{self.y_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
            #ax.text(self.x_interp[frame], -1.4, f'{self.x_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
            
            # Add time label
            ax.text(0, -1.6, f"Time: {self.time_interp[frame]:.2f}s", color='black', fontsize=12, ha='center', va='center')
        
        ellipse_front = Ellipse((0,0),  self.ellipse_1[0], self.ellipse_1[1], angle = 0,        
                  edgecolor='pink', facecolor='pink', alpha = 0.5, linewidth=2)
        self.AddTransformation(ellipse_front, self.angle_interp_1[frame], self.axs[0])
        
        ellipse_lat = Ellipse((0,0),  self.ellispe_2[0], self.ellispe_2[1], angle = 0,        
                  edgecolor='pink', facecolor='pink', alpha = 0.5, linewidth=2)
        
        self.AddTransformation(ellipse_lat, self.angle_interp_2[frame], self.axs[1])
        
        line_ellispe_1 = Rectangle((-self.line_ellipse[0]/2, -self.line[1]/2), self.line_ellipse[0], self.line[1],
                 color='red')
        self.AddTransformation(line_ellispe_1, self.angle_interp_1[frame], self.axs[0])

        line_ellispe_2 = Rectangle((-self.line_ellipse[1]/2, -self.line[1]/2), self.line_ellipse[1], self.line[1],
                 color='red')
        self.AddTransformation(line_ellispe_2, self.angle_interp_2[frame], self.axs[1])

        # Add angle 
        self.axs[0].text(0, -1.4, f'{180/np.pi*self.angle_interp_1[frame]:.0f}°', color='red', fontsize=15, ha='center', va='center')
        self.axs[1].text(0, -1.4, f'{180/np.pi*self.angle_interp_2[frame]:.0f}°', color='red', fontsize=15, ha='center', va='center')
           
    def Run(self):
        #self.CleanTerminal()
        self.ani = FuncAnimation(self.fig, self.UpdateFigure, interval=50, cache_frame_data=False)
        plt.show()

if __name__ == '__main__':
    try: 
        sliding_interface = RollingInterface(excel_file, test)
        sliding_interface.CleanTerminal()
        #while not rospy.is_shutdown():
        sliding_interface.Run()
    except rospy.ROSInterruptException: 
        pass