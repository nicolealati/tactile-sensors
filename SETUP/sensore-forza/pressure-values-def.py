import numpy as np
import matplotlib.pyplot as plt

def generate_signal(pressure_values, rise_values, stay_time, freq=250):
    # Time step (1 / frequency)
    dt = 1 / freq
    
    # List to hold the resulting signal
    signal = []
    time = []
    
    # Iterate over each z value
    for z in stay_time:
        for i, y in enumerate(rise_values):
            for j, x in enumerate(pressure_values):
                # Calculate the adjusted rise time based on the previous logic
                if i == 0:
                    rise_time_duration = y * x / pressure_values[0]  # For the first y, use the ratio of current x to first x
                else:
                    rise_time_duration = y * (x / pressure_values[0])  # Scale subsequent x values accordingly
                
                # Idle time before the rise: 5 seconds of 0 intensity
                idle_time = np.zeros(int(5 * freq))  # 5 seconds of 0
                
                # Generate rising phase
                rise_time = np.linspace(0, x, int(rise_time_duration * freq))  # Linear rise from 0 to x
                
                # Hold constant phase
                constant_time = np.full(int(z * freq), x)  # Hold the value x for z seconds
                
                # Generate falling phase
                fall_time = np.linspace(x, 0, int(rise_time_duration * freq))  # Linear fall from x to 0
                
                # Concatenate all phases together
                full_signal = np.concatenate([idle_time, rise_time, constant_time, fall_time, idle_time])
                
                # Append the signal and corresponding time array
                signal.extend(full_signal)
                time.extend(np.linspace(len(time) * dt, len(time) * dt + len(full_signal) * dt, len(full_signal)))
    
    # Convert to numpy arrays for better manipulation
    signal = np.array(signal)
    time = np.array(time)
    
    # Return the final signal and time arrays
    return time, signal

# Example values (force levels x, rise times y, constant times z)
pressure_values = [2, 4, 6] # forza
rise_values = [3, 2, 1] # Start with y = 1 for the first x's, then multiply for the following ones
stay_time = [6]

# Generate the signal
time, signal = generate_signal(pressure_values, rise_values, stay_time)

# Plot the signal to visualize
plt.plot(time, signal)
plt.title("Generated Signal")
plt.grid(False)
plt.xlabel("Time [s]")
plt.ylabel("Intensity")
plt.show()
