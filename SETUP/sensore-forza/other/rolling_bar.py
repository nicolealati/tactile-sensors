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
t = np.array([0, 1, 2, 5])  # Time steps
x = np.array([0, 1, 1, 0.5])  # Position of the y-axis (X coordinate)
y = np.array([0, 0, 1, 1])  # Position of the x-axis (Y coordinate)

# Interpolate the data for smooth movement
time_interpolated = np.linspace(0, t[-1], 1000)  # 100 interpolated steps
x_interpolated = np.interp(time_interpolated, t, x)
y_interpolated = np.interp(time_interpolated, t, y)

# Ellipse parameters: Scale it to fit between -1 and 1
a, b = 1, 1  # Semi-major and semi-minor axes set to 1 for a unit circle
theta = np.linspace(0, 2 * np.pi, 500)
ellipse_x = a * np.cos(theta)
ellipse_y = b * np.sin(theta)

# Create the figure
fig, ax = plt.subplots(figsize=(8, 8))

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

    # Re-plot the ellipse
    ax.plot(ellipse_x, ellipse_y, color='lightblue')
    ax.fill(ellipse_x, ellipse_y, color='lightblue', alpha=0.5)

    # Plot the red lines representing the current X and Y axis positions
    ax.axhline(y_interpolated[frame], color='red', linewidth=2, linestyle='-')
    ax.axvline(x_interpolated[frame], color='red', linewidth=2, linestyle='-')
    
    # Add text annotations for the axes' positions
    ax.text(-1.5, y_interpolated[frame]+0.1, f'{y_interpolated[frame]:.2f}', color='red', fontsize=10, ha='center', va='top')
    ax.text(x_interpolated[frame]+0.1, -1.5, f'{x_interpolated[frame]:.2f}', color='red', fontsize=10, ha='left', va='center')
    
    # Add time label
    elapsed_time = time_interpolated[frame]
    ax.text(0, -1.6, f"Time: {elapsed_time:.2f}s", color='black', fontsize=12, ha='center', va='center')
    
    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("ROLLING (bar)")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.axis(False)
    ax.grid(False)

# Create the animation
ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)  # Short interval to check frame updates frequently
plt.show()

