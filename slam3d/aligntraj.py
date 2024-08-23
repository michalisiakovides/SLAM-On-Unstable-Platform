import pandas as pd

# Load the data from tum.txt
tum_df = pd.read_csv('tum.txt', delim_whitespace=True, header=None, names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

# Define the start and end of the gzgzf.txt time range
gzgzf_start = 136.883
gzgzf_end = 334.213

# Define the range of tum.txt timestamps
tum_start = 1
tum_end = 59

# Calculate the scaling factor and offset
time_range_gzgzf = gzgzf_end - gzgzf_start
time_range_tum = tum_end - tum_start
scaling_factor = time_range_gzgzf / time_range_tum
offset = gzgzf_start - tum_start * scaling_factor

# Convert and align timestamps
tum_df['timestamp'] = tum_df['timestamp'] * scaling_factor + offset

# Save the aligned tum.txt
tum_df.to_csv('tum_aligned.txt', sep=' ', header=False, index=False)

# Print a few entries to verify
print(tum_df.head())
print(tum_df.tail())

