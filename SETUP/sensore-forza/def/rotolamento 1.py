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
t = np.array([0, 1, 3, 5, 6, 8, 15])  # Time steps
x = np.array([0, 0.5, 0.5, 1, 0, 1, 0])  # Position of the y-axis (X coordinate)
y = np.array([0, 0, 1, 1, 0.5, 0.5, 0])  # Position of the x-axis (Y coordinate)

# Interpolate the data for smooth movement
time_interpolated = np.linspace(0, t[-1], 100)  # Use the last time value from `t`
x_interpolated = np.interp(time_interpolated, t, x)  # Interpolated X positions
y_interpolated = np.interp(time_interpolated, t, y)  # Interpolated Y positions

# Ellipse parameters: Scale it to fit between -1 and 1
a, b = 1, 1  # Semi-major and semi-minor axes set to 1 for a unit circle
theta = np.linspace(0, 2 * np.pi, 1000)

ellipse_x = a * np.cos(theta)
ellipse_y = b * np.sin(theta)

# Create the figure
fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(ellipse_x, ellipse_y, color='lightblue')
ax.fill(ellipse_x, ellipse_y, color='lightblue', alpha=0.5)

# Define the function for updating the plot
def update(frame):
    ax.clear()
    
    # Re-plot the ellipse
    ax.plot(ellipse_x, ellipse_y, color='lightblue')
    ax.fill(ellipse_x, ellipse_y, color='lightblue', alpha=0.5)

    # Plot the red lines representing the current X and Y axis positions
    ax.axhline(y_interpolated[frame], color='red', linewidth=1, linestyle='-')
    ax.axvline(x_interpolated[frame], color='red', linewidth=1, linestyle='-')
    
    # Add text annotations for the axes' positions
    ax.text(-1.1, y_interpolated[frame], f'{y_interpolated[frame]:.2f}', color='red', fontsize=10, ha='left', va='center')
    ax.text(x_interpolated[frame], -1.1, f'{x_interpolated[frame]:.2f}', color='red', fontsize=10, ha='center', va='bottom')
    
    # Add time label
    elapsed_time = time_interpolated[frame]
    ax.text(0.9, -1.1, f"Time: {elapsed_time:.2f}s", color='black', fontsize=12, ha='right', va='bottom')
    
    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Axis Movement Animation")
    
    # Set axis limits to [-1, 1] for both X and Y axes
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    
    # Disable axis lines
    ax.axis(False)
    
    # Grid
    ax.grid(True)

    # Stop the animation at the final frame
    if frame == len(time_interpolated) - 1:
        ani.event_source.stop()

# Create the animation
ani = FuncAnimation(fig, update, frames=len(time_interpolated), interval=50)

plt.show()
