import numpy as np
import matplotlib.pyplot as plt

def generate_variable_impulse_times():
    impulse_times = []
    time = 0
    
    # First phase: every 1.5 seconds for 30 seconds
    interval = 1.5
    end_time = 30
    while time < end_time:
        impulse_times.append(time)
        time += interval
    phase1_end = time
    
    # Second phase: reduce interval from 1.45 to 1.0 seconds
    intervals = np.arange(1.45, 1.0 - 0.05, -0.05)
    for interval in intervals:
        if time >= 60:  # 30 seconds from the previous end time
            break
        impulse_times.append(time)
        time += interval
    phase2_end = time
    
    # Third phase: every 1.0 seconds for 30 seconds
    start_time = time
    end_time = start_time + 30
    while time < end_time:
        impulse_times.append(time)
        time += 1.0
    phase3_end = time
    
    # Fourth phase: reduce interval from 0.95 to 0.5 seconds
    intervals = np.arange(0.95, 0.5 - 0.05, -0.05)
    for interval in intervals:
        if time >= end_time + 30:  # Another 30 seconds
            break
        impulse_times.append(time)
        time += interval
    phase4_end = time
    
    # Fifth phase: every 0.5 seconds for 30 seconds
    start_time = time
    end_time = start_time + 30
    while time < end_time:
        impulse_times.append(time)
        time += 0.5
    phase5_end = time
    
    return impulse_times, [phase1_end, phase2_end, phase3_end, phase4_end, phase5_end]

def generate_impulse_signal(impulse_times, max_amplitude=8, freq=250):
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

# Generate the impulse times and phase end times
impulse_times, phase_end_times = generate_variable_impulse_times()

# Generate the impulse signal
time, signal = generate_impulse_signal(impulse_times)

# Plot the signal to visualize
plt.plot(time, signal)
plt.title("Variable Impulse Signal")
plt.grid(True)
plt.xlabel("Time [s]")
plt.ylabel("Intensity")

# Add red vertical lines to divide the phases
for phase_end_time in phase_end_times:
    plt.axvline(x=phase_end_time, color='red', linestyle='--', linewidth=2)

plt.show()

print("Impulse times:", impulse_times)