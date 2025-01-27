import itertools
import random
import numpy as np
import os
import pandas as pd

test = 'rolling'
dir = rf"D:\GitHub\tactile-sensors\SETUP\sensore-forza\sequences\{test}"
filename = f"{test}_sequence.npy"

############

# Define variables
n_reps = 6
dur = [1, 2]
if test == 'sliding': 
    trigger_index = [0, 1, 2, 3, 4, 5, 6]
else: 
    trigger_index = [0, 5, 6, 7, 8]


# Function to apply the rule not to repeat certain values consecutively
def is_valid_sequence(sequence):
    last_seen = {
        1: -1,
        2: -1,
        3: -1,
        4: -1,
        5: -1,
        6: -1,
        7: -1,
        8: -1
    }

    for i in range(1, len(sequence)):
        curr_value = sequence[i][0]
        prev_value = sequence[i-1][0]

        # Check for consecutive repetition of the same value
        if curr_value == prev_value:
            return False

        # Check for the new rules about distances between pairs
        if curr_value == 1 and last_seen[1] != -1 and last_seen[1]>last_seen[2]:
            return False
        if curr_value == 2 and last_seen[2] != -1 and last_seen[2]>last_seen[1]:
            return False
        if curr_value == 3 and last_seen[3] != -1 and last_seen[3]>last_seen[4]:
            return False
        if curr_value == 4 and last_seen[4] != -1 and last_seen[4]>last_seen[3]:
            return False
        if curr_value == 5 and last_seen[5] != -1 and last_seen[5]>last_seen[6]:
            return False
        if curr_value == 6 and last_seen[6] != -1 and last_seen[6]>last_seen[5]:
            return False
        if curr_value == 7 and last_seen[7] != -1 and last_seen[7]>last_seen[8]:
            return False
        if curr_value == 8 and last_seen[8] != -1 and last_seen[8]>last_seen[7]:
            return False

        # Update the last seen index of the current value
        last_seen[curr_value] = i

    return True

# Function to generate a valid sequence of 6 repetitions
def generate_valid_sequence(pairs, repetitions=1):
    # Repeat the pairs for the required number of repetitions
    extended_pairs = pairs*repetitions

    # Try to mix the pairs until we get a valid sequence
    random.shuffle(extended_pairs)
    
    # Check that the sequence is valid
    while not is_valid_sequence(extended_pairs):
        random.shuffle(extended_pairs)
    #print("found sequence")
    
    return extended_pairs

#######
if os.name == 'nt':
    os.system('cls')

# Create a valid sequence
valid_sequence=[0]*n_reps
pairs = list(itertools.product(trigger_index, dur))

# Generate the valid sequence
for rep in range(0, n_reps):
    valid_sequence[rep] = generate_valid_sequence(pairs)

# Print the sequence
for seq in valid_sequence:
    # Calculate the sum of values in `t`s (the second column of pairs)
    sum_t = sum(pair[1] for pair in seq)
    #print(f"Sum of t values: {sum_t}")

# Concatenate sequence
concatenated_sequence = [pair for seq in valid_sequence for pair in seq]
sequence_def = np.array(concatenated_sequence)
np.save(f"{dir}/{filename}", sequence_def)
print(f"Sequence saved\n")

