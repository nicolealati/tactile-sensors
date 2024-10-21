import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqs
import matplotlib.pyplot as plt
import numpy as np

#load signal from C:\Users\lar\Documents\Signals\Cube sliding
signal_path = 'C:/Users/lar/Documents/Signals/Cube stationary/'
thumb = np.load(signal_path + 'thumb.npy')
index = np.load(signal_path + 'index.npy')
middle = np.load(signal_path + 'middle.npy')
ring = np.load(signal_path + 'ring.npy')
little = np.load(signal_path + 'little.npy')
finger = thumb
taxel = 2
freq = 310
#filter finger with high butter filter
def butter_highpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high')
    return b, a

def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

cutoff = 50

finger_filtered = butter_highpass_filter(finger, cutoff, freq, order=4)

integral_finger = np.zeros(finger.shape[0])
for i in range(1,finger.shape[0]):
    integral_finger[i] = integral_finger[i-1] + finger[i,taxel]/freq

integral_finger_filtered = np.zeros(finger.shape[0])
for i in range(1,finger.shape[0]):
    integral_finger_filtered[i] = integral_finger_filtered[i-1] + finger_filtered[i,taxel]/freq

#i want the mean of the signal finger
mean_finger = np.mean(finger, axis=0)
print(mean_finger)
#i want to calculate the integral of the finger subtracting the mean
integral_finger_mean = np.zeros(finger.shape[0])
for i in range(1,finger.shape[0]):
    integral_finger_mean[i] = integral_finger_mean[i-1] + (finger[i,taxel]-mean_finger[taxel])/freq


#plot signal of finger and summed with legend
plt.plot(finger[:,taxel])
plt.plot(finger_filtered[:,taxel])
plt.plot(integral_finger)
plt.plot(integral_finger_filtered)
plt.plot(integral_finger_mean)
plt.legend(['thumb', 'summed', 'integral', 'integral filtered', 'integral mean'])
plt.show()

