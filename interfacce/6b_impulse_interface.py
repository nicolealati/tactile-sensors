import matplotlib.pyplot as plt
import numpy as np
import time
import os

if os.name == 'nt':
    os.system('cls')

class ImpulseInterface:
    def __init__(self):
        self.test = "impulse"
        self.load_dir = rf"D:\GitHub\tactile-sensors\SETUP\sensore-forza\sequences\{self.test}"
        self.filename = f"{self.test}_sequence.npy"

        self.loaded_signal = np.load(f"{self.load_dir}\{self.filename}")
        
        self.time = self.loaded_signal[0]
        self.signal = self.loaded_signal[1]
        self.freq = len(self.time)/self.time[-1]

        self.total_frames = len(self.signal)
        self.delta_time = self.time[1]-self.time[0]

        self.window = 6  # Dimensione della finestra (secondi)
        self.current_time = 0  # Tempo corrente (inizio)
        self.start_time = None
        
        self.current_time-self.window/2
        
        self.start_window = self.current_time-self.window/2
        self.end_window = self.current_time+self.window/2
        self.center = (self.start_window+self.end_window)/2
        
        self.fig, self.ax = plt.subplots()

        self.ax.axvspan(self.start_window, self.end_window, color='lightblue', alpha=0.3)
        self.ax.axvspan(self.center-0.1, self.center+0.1, color='red', alpha=0.2)
        self.ax.plot(self.time, self.signal, 'black', linewidth=2)
              
        self.ax.set_xlim(self.start_window, self.end_window)
        self.ax.set_ylim(-2, 1.2*max(self.signal))
        self.ax.set_title("IMPULSE")
        self.ax.set_xticks([]), self.ax.set_yticks([])

        self.ax.text(0, -1.1, f"Time: {self.current_time:.2f}s", color='black', fontsize=12, ha='center', va='center')
        
        plt.ion()
        plt.draw()
        plt.pause(self.delta_time)

    def Run(self):
        plt.ion()

        if self.start_time is None:
            #input("Press Enter to start the task...")
            self.start_time = time.time()
            self.current_time = 0
            print("Interface is running")
   
        while self.current_time<self.time[-1]:
            self.elapsed_time = time.time()-self.start_time
            self.current_time = self.elapsed_time#-self.window #if self.elapsed_time>self.window else 0
            #end_time = self.current_time+self.window

            # Calcola i limiti della finestra centrata in 0
            self.start_window = self.current_time-self.window/2
            self.end_window = self.current_time+self.window/2
            self.center = (self.start_window+self.end_window)/2

            # PLOT
            self.ax.clear()
            
            self.ax.axvspan(self.start_window, self.end_window, color='lightblue', alpha=0.3)
            self.ax.axvspan(self.center-0.1, self.center+0.1, color='red', alpha=0.2)
            
            self.ax.plot(self.time, self.signal, 'black', linewidth=2)

            self.ax.set_xlim(self.start_window, self.end_window)
            self.ax.set_ylim(-1, 1.2*max(self.signal))
            self.ax.set_title("IMPULSE")
            self.ax.set_xticks([]), self.ax.set_yticks([])

            self.ax.text(self.center, -1.5, f"Time: {self.elapsed_time:.2f}s", color='black', fontsize=12, ha='center', va='center')

            plt.draw()
            plt.pause(self.delta_time)

        print("Stop animation")
        plt.ioff()
        plt.close()
        

if __name__ == "__main__":
    interface = ImpulseInterface()
    interface.Run()
