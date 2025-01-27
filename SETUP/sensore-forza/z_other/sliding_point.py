import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os

def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')

CleanTerminal()

# Define the time and positions for the axes
t = np.array([0, 3, 5, 8])  # Time steps
x = np.array([0, 1, 0.3, 0.3])  # Position of the y-axis (X coordinate)
y = np.array([0, 0, 0.8, 0.8])  # Position of the x-axis (Y coordinate)

# Interpolate the data for smooth movement
time_interpolated = np.linspace(0, t[-1], 1000)  # 100 interpolated steps
x_interpolated = np.interp(time_interpolated, t, x)
y_interpolated = np.interp(time_interpolated, t, y)

# Create the figure
fig, ax = plt.subplots(figsize=(8, 8))

# Rectangle
rect_base = 2
rect_height = 2
plt.gca().add_patch(plt.Rectangle((-1, -1), rect_base, rect_height, color='lightblue', alpha=0.5))

# Real-time animation start time
start_time = time.time()

# Define the function for updating the plot
def update(frame):
    ax.clear()
    
    # Calculate the actual elapsed time in seconds
    elapsed_real_time = time.time() - start_time
    # Find the closest frame based on real elapsed time
    frame = np.searchsorted(time_interpolated, elapsed_real_time, side='right') - 1

    # Stop the animation when the last frame is reached
    if frame >= len(time_interpolated) - 1:
        ani.event_source.stop()
        plt.close()
        return

    # Re-plot rectangle
    plt.gca().add_patch(plt.Rectangle((-1, -1), rect_base, rect_height, color='lightblue', alpha=0.5), )

    # Plot the red lines representing the current X and Y axis positions
    ax.plot(x_interpolated[frame], y_interpolated[frame], 'ro', markersize=30)
    
    # Add text annotations for the axes' positions
    ax.text(-1.1, y_interpolated[frame], f'{y_interpolated[frame]:.2f}', color='red', fontsize=10, ha='center', va='center')
    ax.text(x_interpolated[frame], -1.1, f'{x_interpolated[frame]:.2f}', color='red', fontsize=10, ha='left', va='center')
    
    # Add time label
    ax.text(0.0, -1.2, f"Time: {time_interpolated[frame]:.2f}s", color='black', fontsize=12, ha='center', va='center')
    
    # Set labels and title
     
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"SLIDING \npoint")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.axis(False)
    ax.grid(False)
    


# Create the animation
ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)  # Short interval to check frame updates frequently
plt.show()

