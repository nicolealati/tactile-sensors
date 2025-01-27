import numpy as np
import os
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.patches import Ellipse
from matplotlib.transforms import Affine2D

excel_file = "pattern.xlsx"
test = "ROLLING"

class RollingAnimation:
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

        ### 
        # Example current force
        self.force_x = np.linspace(0, 2*np.pi, self.n_frame)
        self.applied_force = self.required_force+np.sin(self.force_x)
        
        '''
        force_amp = (np.cos(force_x)+1)/2
        self.applied_force = np.sin(force_x)*force_amp
        '''    

        # Create the figure
        self.fig, self.axs = plt.subplots(1, 2, figsize=(18, 8))
        for ax in self.axs:
            ax.axis(False)

    def CleanTerminal(self):
        if os.name == 'nt':
            os.system('cls')

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
        
    def AddTransformation(self, figure, x, y, rot, ax):
        if ax is None:
            raise ValueError("Error ax.")

        transform = (Affine2D().rotate(rot).translate(x, y) + ax.transData)
        figure.set_transform(transform)
        ax.add_patch(figure)

    def update(self, frame):
        for ax in self.axs:
            ax.clear()

        # Initialize start_time on the first frame
        if self.start_time is None:
            self.start_time = time.time()

        # Calculate the actual elapsed time in seconds
        elapsed_time = time.time()-self.start_time

        # Find the closest frame based on real elapsed time
        frame = np.searchsorted(self.time_interp, elapsed_time, side='right')-1

        # Stop the animation when the last frame is reached
        if frame >= len(self.time_interp)-1:
            self.ani.event_source.stop()
            plt.close()
            return

        # Retrieve the interpolated values for the current frame
        x_center = self.x_interp[frame]
        y_center = self.y_interp[frame]

        self.trans_force = self.applied_force[frame]-self.required_force
        self.scaled_force = self.trans_force*self.rect[1]/self.tolerance

        # Moving figures
        for ax in self.axs:

            # Ground rectangle
            ax.add_patch(Rectangle((-self.square[0]/2, -self.square[1]/2), self.square[0], self.square[1], color='lightblue', alpha=0.5))

            box = Rectangle((-self.rect[0], -self.rect[1]), self.rect[0]*2, self.rect[1]*2, color='green')
            self.AddTransformation(box, x_center, y_center, 0, ax)

            line = Rectangle((-1.5*self.line[0]-self.line[0]/2, 0), self.line[0]*4, self.line[1], color='black')
            self.AddTransformation(line, x_center, y_center+self.scaled_force, 0, ax)

            # Add text annotations for the axes' positions
            #ax.text(-1.5, self.y_interp[frame], f'{self.y_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
            #ax.text(self.x_interp[frame], -1.4, f'{self.x_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
            
            # Add time label
            ax.text(0, -1.6, f"Time: {self.time_interp[frame]:.2f}s", color='black', fontsize=12, ha='center', va='center')
        
        ellipse_front = Ellipse((0,0),  self.ellipse_1[0], self.ellipse_1[1], angle = 0,        
                  edgecolor='pink', facecolor='pink', alpha = 0.5, linewidth=2)
        self.AddTransformation(ellipse_front, x_center, y_center, self.angle_interp_1[frame], self.axs[0])

        ellipse_lat = Ellipse((0,0),  self.ellispe_2[0], self.ellispe_2[1], angle = 0,        
                  edgecolor='pink', facecolor='pink', alpha = 0.5, linewidth=2)
        self.AddTransformation(ellipse_lat, x_center, y_center, self.angle_interp_2[frame], self.axs[1])

        line_ellispe_1 = Rectangle((-self.line_ellipse[0]/2, -self.line[1]/2), self.line_ellipse[0], self.line[1],
                 color='red')
        self.AddTransformation(line_ellispe_1, x_center, y_center, self.angle_interp_1[frame], self.axs[0])

        line_ellispe_2 = Rectangle((-self.line_ellipse[1]/2, -self.line[1]/2), self.line_ellipse[1], self.line[1],
                 color='red')
        self.AddTransformation(line_ellispe_2, x_center, y_center, self.angle_interp_2[frame], self.axs[1])

        # Add angle 
        self.axs[0].text(0, -1.4, f'{180/np.pi*self.angle_interp_1[frame]:.0f}°', color='red', fontsize=15, ha='center', va='center')
        self.axs[1].text(0, -1.4, f'{180/np.pi*self.angle_interp_2[frame]:.0f}°', color='red', fontsize=15, ha='center', va='center')
        
        # Set titles for each subplot
        self.axs[0].set_title("LATERAL VISION")
        self.axs[1].set_title("FRONTAL VISION")

        for ax in self.axs:
            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.axis(False)

    def Run(self):
        self.CleanTerminal()
        self.ani = FuncAnimation(self.fig, self.update, interval=50, cache_frame_data=False)
        plt.show()

if __name__ == '__main__':
    animation = RollingAnimation(excel_file, test)
    animation.Run()
