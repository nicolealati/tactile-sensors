import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from scipy.signal import freqs

taxel = 2
dir = "/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/PushtoThumb"
thumb = np.load(dir + "/thumb.npy")
index = np.load(dir + "/index.npy")
middle = np.load(dir + "/middle.npy")
ring = np.load(dir + "/ring.npy")
little = np.load(dir + "/little.npy")

finger=thumb

#detrend lineare del segnale
x = np.arange(len(finger[:,taxel]))
z = np.polyfit(x, finger[:,taxel], 2)
p = np.poly1d(z)
new_finger = finger[:,taxel] - p(x)

new_finger_zeroed = new_finger
for i in range(0, len(new_finger)):
    if abs(new_finger[i]) < 5:
        new_finger_zeroed[i] = 0

#std of the signal forward and backward
new_finger_std_forward = np.zeros(finger.shape[0])
new_finger_std_backward = np.zeros(finger.shape[0])

window = 300
for i in range(0, len(new_finger)):
    if i < window:
        new_finger_std_forward[i] = np.std(new_finger[:i])
    else:
        new_finger_std_forward[i] = np.std(new_finger[i-window:i])
    if i > len(new_finger) - window:
        new_finger_std_backward[i] = np.std(new_finger[i:])
    else:
        new_finger_std_backward[i] = np.std(new_finger[i:i+window])

new_finger_std = np.minimum(new_finger_std_forward, new_finger_std_backward)

#integral of the signal
integral = np.cumsum(new_finger)

detrending_curve = np.zeros(finger.shape[0])
for i in range(0, len(detrending_curve)):
    if new_finger_std[i] == 0:
        detrending_curve[i] = integral[i]

integral = integral - detrending_curve

# Plotting the signals
fig, axs = plt.subplots(2, 1)
axs[0].plot(new_finger_zeroed)
axs[0].plot(new_finger_std, label='STD')
axs[0].set_title('Derivative signal')
axs[1].plot(integral, label='Adjusted Integral')
axs[1].set_title('Force signal')
axs[1].grid()
axs[1].legend()
plt.show()

fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # Adjust figure size for better clarity

# Plot the zeroed signal with standard deviation
axs[0].plot(new_finger_zeroed, label='Zeroed Signal', color='b')
axs[0].plot(new_finger_std, label='Standard Deviation (STD)', color='r', linestyle='--')
axs[0].set_title('Detrended and Zeroed Signal with STD', fontsize=14)
axs[0].set_xlabel('Time (samples)', fontsize=12)
axs[0].set_ylabel('Amplitude', fontsize=12)
axs[0].legend(loc='upper right')
axs[0].grid(True)

# Plot the adjusted integral signal
axs[1].plot(integral, label='Adjusted Integral', color='g')
axs[1].set_title('Adjusted Integral of the Detrended Signal', fontsize=14)
axs[1].set_xlabel('Time (samples)', fontsize=12)
axs[1].set_ylabel('Integral Value', fontsize=12)
axs[1].grid(True)
axs[1].legend(loc='upper right')

# Tighten the layout
plt.tight_layout()

# Show the plot
plt.show()
#write a column csv file with the original signal
np.savetxt(dir + "/thumb.csv", finger[:,taxel], delimiter=",")