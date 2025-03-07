import numpy as np
import matplotlib.pyplot as plt

# Given values
initial_position = 130  # starting position
desired_position = 90    # target position
velocity = 5.1           # velocity
acceleration = -6        # deceleration (since it's slowing down)

# Time of motion calculation based on kinematic equation: v^2 = u^2 + 2as
distance = desired_position - initial_position
time_to_stop = 10.0

# Generate time values
time_values = np.linspace(0, time_to_stop, 100)

# Position equation: s = s0 + ut + 0.5at^2
position_values = initial_position + velocity * time_values + 0.5 * acceleration * time_values**2

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(time_values, position_values, label="Position vs Time", color='b')
plt.title("Position vs Time (Chậm Dần Đều)")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.grid(True)
plt.legend()
plt.show()
