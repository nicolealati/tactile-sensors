import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os
os.system('cls')

type = "raw"

pred_path = f"pressure hand {type} - pred.npy"
true_path = f"pressure hand {type} - true.npy"
pred_signal = -np.load(pred_path)
true_signal = -np.load(true_path)

freq = 313
pred_time = np.linspace(0, len(pred_signal)-1, len(pred_signal))*1/freq
true_time = np.linspace(0, len(true_signal)-1, len(true_signal))*1/freq

print("(PRED) Length = ", len(pred_signal), ' - Time = ', len(pred_time), 'Dur = ', pred_time[-1])
print("(TRUE) Length = ", len(true_signal), ' - Time = ', len(true_time), 'Dur = ', true_time[-1])

piezo_path = "piezo_index.npy"
piezo_signal = np.load(piezo_path)
piezo_time = np.linspace(0, len(piezo_signal)-1, len(piezo_signal))*1/freq
piezo_time = np.linspace(0, piezo_time[-1], len(piezo_signal))
piezo_signal = interp1d(piezo_time, piezo_signal, axis=0, kind='linear')
piezo_signal = piezo_signal(piezo_time)

print("(PIEZO) Length = ", len(piezo_signal), ' - Time = ', len(piezo_time), 'Dur = ', piezo_time[-1])

# Plot 
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(piezo_time, piezo_signal)

plt.subplot(2, 1, 2) 
plt.plot(pred_time, pred_signal, label = 'pred')
plt.plot(true_time, true_signal, label = 'true')
plt.title(type)
plt.grid(True)
plt.legend()
plt.show()

# Save txt
np.savetxt(f"{type} hand pred_signal.txt", pred_signal)
np.savetxt(f"{type} hand pred_time.txt", pred_time)
np.savetxt(f"{type} hand true_signal.txt", true_signal)
np.savetxt(f"{type} hand true_time.txt", true_time)

np.savetxt(f"piezo hand signal.txt", piezo_signal)
np.savetxt(f"piezo hand time.txt", piezo_time)

# txt_true_path = true_path.replace(".npy", ".txt")
# np.savetxt(txt_true_path, [true_time, true_signal])

