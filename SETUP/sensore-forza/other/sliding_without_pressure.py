import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

# Clean terminal
def CleanTerminal():
    if os.name == 'nt':
        _ = os.system('cls')
CleanTerminal()

# Define the time and positions for the axes
t = np.array([0, 2, 5, 8, 10])  # Time steps
x = np.array([0, 1, 1, 1, 1])  # Position of the y-axis (X coordinate)
y = np.array([0, 0, 0, 1, 1])  # Position of the x-axis (Y coordinate)
angle = np.array([0, 0, np.pi/2, np.pi/2, 0])  # Orientation of the rectangle in radians

# Interpolate the data for smooth movement
n = 1000
time_interpolated = np.linspace(0, t[-1], n)  # 1000 interpolated steps
x_interpolated = np.interp(time_interpolated, t, x)
y_interpolated = np.interp(time_interpolated, t, y)
angle_interpolated = np.interp(time_interpolated, t, angle)

# Create the figure
fig, ax = plt.subplots(figsize=(8, 8))

# Rectangle
rect_base = 2.4
rect_height = 2.4
plt.gca().add_patch(plt.Rectangle((-rect_base/2, -rect_height/2), rect_base, rect_height, color='lightblue', alpha=0.5))

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
    plt.gca().add_patch(plt.Rectangle((-rect_base/2, -rect_height/2), rect_base, rect_height, color='lightblue', alpha=0.5))

    # Plot the small red rectangle with rotation
    small_rect_base = 0.2
    small_rect_height = 0.4
    center_x = x_interpolated[frame]
    center_y = y_interpolated[frame]
    rect = Rectangle(
        (-small_rect_base / 2, -small_rect_height / 2),
        small_rect_base,
        small_rect_height,
        color='red'
    )
    # Apply transformation for rotation around the center
    transform = (
        Affine2D()
        .rotate(angle_interpolated[frame])
        .translate(center_x, center_y)
        + ax.transData
    )
    rect.set_transform(transform)
    ax.add_patch(rect)

    # Add text annotations for the axes' positions
    ax.text(-1.1, y_interpolated[frame], f'{y_interpolated[frame]:.2f}', color='red', fontsize=10, ha='left', va='center')
    ax.text(x_interpolated[frame], -1.2, f'{x_interpolated[frame]:.2f}', color='red', fontsize=10, ha='center', va='bottom')
    
    # Add time label
    ax.text(0.9, -1.1, f"Time: {time_interpolated[frame]:.2f}s", color='black', fontsize=12, ha='right', va='bottom')
    
    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"SLIDING\nw/o pressure")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.axis(False)
    ax.grid(False)

# Create the animation
ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)  # Short interval to check frame updates frequently
plt.show()
