import numpy as np
import matplotlib.pyplot as plt

def generate_impulse_signal(x_vals, impulse_times, max_amplitude=8, freq=250):
    # Time step (1 / frequency)
    dt = 1 / freq
    
    # Create the signal array initialized to 0
    total_duration = int(np.ceil(max(impulse_times)) + 1)  # Duration based on the latest impulse time
    signal = np.zeros(int(total_duration * freq))  # Initialize signal array with zeros
    
    # Create a time array corresponding to the signal
    time = np.linspace(0, total_duration, len(signal))
    
    # Iterate over each impulse time and set the corresponding signal value
    for impulse_time in impulse_times:
        # Find the index in the signal array corresponding to the impulse time
        index = int(np.round(impulse_time * freq))
        if 0 <= index < len(signal):
            signal[index] = max_amplitude  # Set the impulse value
    
    # Return the time and signal arrays
    return time, signal

# Example: Define the impulse times
impulse_times = [4, 6, 7, 9, 13, 15, 17]

# Generate the impulse signal
time, signal = generate_impulse_signal(x_vals=None, impulse_times=impulse_times)

# Plot the signal to visualize
plt.plot(time, signal)
plt.title("Impulse Signal")
plt.grid(False)
plt.xlabel("Time [s]")
plt.ylabel("Intensity")
plt.show()
