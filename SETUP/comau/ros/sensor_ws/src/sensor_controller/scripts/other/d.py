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
z = np.polyfit(x, finger[:,taxel], 1)
p = np.poly1d(z)
new_finger = finger[:,taxel] - p(x)

#integral of the signal
det_integral = np.cumsum(new_finger)
integral_original = np.cumsum(finger[:,taxel])

#detrending non lineare del segnale
x = np.arange(len(integral_original))
z = np.polyfit(x, integral_original, 3)
p = np.poly1d(z)
integral_det = integral_original - p(x)

new_integral = np.zeros(finger.shape[0])

# Apply the logic to track increases and decreases, but ensure new_integral doesn't go below zero
for e in range(1, len(integral_det)):
    delta = integral_det[e] - integral_det[e - 1]
    new_integral[e] = max(0, new_integral[e - 1] + delta)


# Plotting the signals
fig, axs = plt.subplots(2, 1)
axs[0].plot(new_finger)
axs[0].set_title('Derivative signal')
axs[1].plot(integral_original, label='Original Integral')
axs[1].plot(p(x), label='Trend')
axs[1].plot(new_integral, label='Adjusted Integral (Rising)')
axs[1].set_title('Force signal')
axs[1].grid()
axs[1].legend()
plt.show()


#write a column csv file with the original signal
np.savetxt(dir + "/thumb.csv", finger[:,taxel], delimiter=",")