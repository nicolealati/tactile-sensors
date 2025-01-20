import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, filtfilt

taxel = 6
dir = "/home/alex/ros/Sensor_ws/src/sensor_controller/scripts/records/Sphere stationary"
thumb = np.load(dir + "/thumb.npy")
index = np.load(dir + "/index.npy")
middle = np.load(dir + "/middle.npy")
ring = np.load(dir + "/ring.npy")
little = np.load(dir + "/little.npy")

finger=middle

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data, axis=0)  # Apply filter along the time axis
    return y

# Define filter parameters
fs = 310  # Assumed sampling frequency in Hz (based on previous code)
cutoff = 30
order = 2

# Apply the low-pass filter to reduce noise
new_finger = butter_lowpass_filter(finger, cutoff, fs, order)

# Plot original vs filtered signal for one taxel (index 0) as an example
# plt.figure(figsize=(12, 6))
# plt.plot(finger[:2000, 0], label='Original Signal', alpha=0.5)
# plt.plot(new_finger[:2000, 0], label='Filtered Signal', color='red')
# plt.title('Original vs Filtered Signal (Taxel 0)')
# plt.xlabel('Sample')
# plt.ylabel('Force Variation')
# plt.legend()
# plt.show()

# force_signal = np.cumsum(new_finger, axis=0) * (1/fs)  # Integrating over time (cumulative sum)

# Plot the integrated signal (force) for one taxel (index 0) as an example
# plt.figure(figsize=(12, 6))
# plt.plot(force_signal[:2000, 0], label='Estimated Force (Taxel 0)', color='green')
# plt.title('Estimated Force from Filtered Signal (Taxel 0)')
# plt.xlabel('Sample')
# plt.ylabel('Estimated Force')
# plt.legend()
# plt.show()

#otteniamo l'integrale del segnale filtrato
freq = 25
integral = np.zeros(finger.shape)
integral[0] = new_finger[0]/freq
for i in range(1, new_finger.shape[0]):
    integral[i] = integral[i-1] + new_finger[i]/freq
    if integral[i,taxel] < 0:
        integral[i,taxel] = 0

# #i want to subract the mean after every 100 samples
# for i in range(0, integral.shape[0], 1000):
#     mean = np.mean(integral[i:i+1000], axis=0)
#     integral[i:i+1000] -= mean
# for i in range(1, new_finger.shape[0]):
#     integral[i,taxel] = abs(integral[i,taxel])
        
    # for j in range(1, integral[i].shape[0]):
    #     if integral[i][j] < 0:
    #         integral[i][j] = 0

plt.figure()
plt.plot(finger[:,taxel], alpha=0.5)
plt.plot(new_finger[:,taxel], alpha=0.6)
plt.plot(integral[:,taxel])
plt.xlabel("Samples")
plt.ylabel("Magnitude")
plt.title("FInger signal integral")
plt.grid()
plt.legend(["Original", "Filtered", "Integral"])
plt.show()
