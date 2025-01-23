import numpy as np
import os
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

excel_file = "pattern.xlsx"
test = "SLIDING"

class SlidingInterface:
    def __init__(self, excel_file, sheet_name):
        
        self.excel_file = excel_file
        self.sheet_name = sheet_name
        self.start_time = None
        self.trigger = [[1],      # stop
                        [0], [0], # destra / sinistra
                        [0], [0], # alto / basso
                        [0], [0]] # orario / antiorario
        self.save_dir = r"D:\GitHub\tactile-sensors\SETUP\sensore-forza\data\trigger_sliding"
        
        # Figures dimensions
        self.square = [2.5, 2.5]
        self.rect = [0.1, 0.25]
        self.line = [self.rect[0], 0.01]

        self.n_frame = 10000

        # Load patterns from excel file
        self.df_pattern = pd.read_excel(self.excel_file, sheet_name=self.sheet_name)

        self.LoadPatterns()
        self.CalculateInterpolation()
        self.CreateTriggers()
        self.SaveData()

        '''        
        print(len(self.time_interp))
        print(len(self.x_interp))
        print(len(self.y_interp))
        print(len(self.angle_interp))

        for i in range(len(self.trigger)): 
            print([i, len(self.trigger[i])])
        '''        

        ### 
        # Example current force
        self.force_x = np.linspace(0, 2*np.pi, self.n_frame)
        self.applied_force = self.required_force+np.sin(self.force_x)
        '''
        force_amp = (np.cos(force_x)+1)/2
        self.applied_force = np.sin(force_x)*force_amp
        '''    

        # Create the figure
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.axis(False)

    def CleanTerminal(self):
        if os.name == 'nt':
            os.system('cls')

    def LoadPatterns(self): 
        self.time_step = np.array(self.df_pattern['time_steps'].astype(int))
        self.x_coord = np.array(self.df_pattern['x_position'].astype(int))
        self.y_coord = np.array(self.df_pattern['y_position'].astype(int))
        self.angle_rot = np.array(self.df_pattern['rotation_angle'].astype(int))/180*np.pi
        self.required_force = np.array(self.df_pattern['required_force'].astype(float))
        self.required_force = self.required_force[~np.isnan(self.required_force)]
        self.tolerance = np.array(self.df_pattern['tolerance'].astype(float))
        self.tolerance = self.tolerance[~np.isnan(self.tolerance)]

    def CalculateInterpolation(self):
        self.time_interp = np.linspace(0, self.time_step[-1], self.n_frame)
        self.x_interp = np.interp(self.time_interp, self.time_step, self.x_coord)
        self.y_interp = np.interp(self.time_interp, self.time_step, self.y_coord)
        self.angle_interp = np.interp(self.time_interp, self.time_step, self.angle_rot)
    
    def CreateTriggers(self): 
        
        # Define trigger from the second frame
        for i in range(len(self.time_interp)-1): 

            # Scivolamento a destra [1] / sinistra [2]
            if self.x_interp[i+1] == self.x_interp[i]: 
                self.trigger[1].append(0)
                self.trigger[2].append(0)
            else:
                self.trigger[1].append(1 if self.x_interp[i+1]>self.x_interp[i] else 0) 
                self.trigger[2].append(1 if self.x_interp[i+1]<self.x_interp[i] else 0)
            
            # Scivolamento in alto [3] / basso [4]
            if self.y_interp[i+1] == self.y_interp[i]: 
                self.trigger[3].append(0)
                self.trigger[4].append(0)
            else:
                self.trigger[3].append(1 if self.y_interp[i+1]>self.y_interp[i] else 0) 
                self.trigger[4].append(1 if self.y_interp[i+1]<self.y_interp[i] else 0)

            # Scivolamento in senso orario [5] / antiorario [6]
            if self.angle_interp[i+1] == self.angle_interp[i]: 
                self.trigger[5].append(0)
                self.trigger[6].append(0)
            else:
                self.trigger[5].append(1 if self.angle_interp[i+1]<self.angle_interp[i] else 0) 
                self.trigger[6].append(1 if self.angle_interp[i+1]>self.angle_interp[i] else 0)            
            
            # Stop [0]
            sum_trigger = 0; 
            for e in range(len(self.trigger)-1): 
                sum_trigger += self.trigger[e+1][i+1]
            self.trigger[0].append(1 if sum_trigger==0 else 0)


    def SaveData(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
                
        np.save(self.save_dir + "/" + "trigger_stop.npy", self.trigger[0])
        np.save(self.save_dir + "/" + "trigger_destra.npy", self.trigger[1])
        np.save(self.save_dir + "/" + "trigger_sinistra.npy", self.trigger[2])
        np.save(self.save_dir + "/" + "trigger_alto.npy", self.trigger[3])
        np.save(self.save_dir + "/" + "trigger_basso.npy", self.trigger[4])
        np.save(self.save_dir + "/" + "trigger_orario.npy", self.trigger[5])
        np.save(self.save_dir + "/" + "trigger_antiorario.npy", self.trigger[6])
        np.save(self.save_dir + "/" + "x_interp.npy", self.x_interp)
        np.save(self.save_dir + "/" + "y_interp.npy", self.y_interp)
        np.save(self.save_dir + "/" + "angle_interp.npy", self.angle_interp)
        np.save(self.save_dir + "/" + "time_interp.npy", self.time_interp) 
        
    def AddTransformation(self, figure, x, y, rot):
        if self.ax is None:
            raise ValueError("Error ax.")

        transform = (Affine2D().rotate(rot).translate(x, y)+self.ax.transData)
        figure.set_transform(transform)
        self.ax.add_patch(figure)

    def update(self, frame):
        self.ax.clear()

        # Initialize start_time on the first frame
        if self.start_time is None:
            self.start_time = time.time()   

        # Calculate the actual elapsed time in seconds and Find the closest frame based on real elapsed time
        elapsed_time = time.time()-self.start_time
        frame = np.searchsorted(self.time_interp, elapsed_time, side='right')-1    

        # Stop the animation when the last frame is reached
        if frame >= len(self.time_interp)-1:
            self.ani.event_source.stop()
            plt.close()
            return

        # Retrieve the interpolated values for the current frame
        x_center = self.x_interp[frame]
        y_center = self.y_interp[frame]

        # Ground rectangle
        self.ax.add_patch(Rectangle((-self.square[0]/2, -self.square[1]/2), self.square[0], self.square[1], color='lightblue', alpha=0.5))

        # Moving figures
        box = Rectangle((-self.rect[0], -self.rect[1]), self.rect[0]*2, self.rect[1]*2, color='green')
        self.AddTransformation(box, x_center, y_center, 0)

        self.trans_force = self.applied_force[frame]-self.required_force
        self.scaled_force = self.trans_force*self.rect[1]/self.tolerance
    
        line = Rectangle((-1.5*self.line[0]-self.line[0]/2, 0), self.line[0]*4, self.line[1], color='black')
        self.AddTransformation(line, x_center, y_center+self.scaled_force, 0)

        rect = Rectangle((-self.rect[0]/2, -self.rect[1]/2), self.rect[0], self.rect[1], color='red', alpha=0.7)
        self.AddTransformation(rect, x_center, y_center, self.angle_interp[frame])

        # Add text annotations for the axes' positions
        self.ax.text(-1.5, self.y_interp[frame], f'{self.y_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
        self.ax.text(self.x_interp[frame], -1.4, f'{self.x_interp[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')

        # Add time label
        self.ax.text(0, -1.6, f"Time: {self.time_interp[frame]:.2f}s", color='black', fontsize=12, ha='center', va='center')

        # Set labels and title
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("SLIDING")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.axis(False)
        self.ax.grid(False)

    def Run(self):
        #self.CleanTerminal()
        self.ani = FuncAnimation(self.fig, self.update, interval=10, cache_frame_data=False)
        plt.show()

if __name__ == '__main__':
    animation = SlidingInterface(excel_file, test)
    animation.Run()
