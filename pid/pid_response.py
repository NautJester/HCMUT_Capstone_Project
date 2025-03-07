import numpy as np
import matplotlib.pyplot as plt

# Define PID controller class
class PID_Controller:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.dt = dt
        self.output = 0

    def compute(self, current_value):
        # Calculate error
        error = self.setpoint - current_value

        # I_error
        self.integral += ((error + self.prev_error) / 2) * self.dt

        # D_error
        derivative = (error - self.prev_error) / self.dt

        # Output
        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update prev_error
        self.prev_error = error

        return self.output

# Parameters
Kp = 1
Ki = 0.7
Kd = 0.01
setpoint =  109.2
current_value = 102.375
dt = 0.005
iterations = 20000  # Number of iterations to simulate

# Initialize PID controller
pid = PID_Controller(Kp, Ki, Kd, setpoint, dt)

# Lists to store values for plotting
time_vals = np.arange(0, iterations*dt, dt)
current_values = np.zeros(iterations)
pid_outputs = np.zeros(iterations)

# Simulate PID control
for i in range(iterations):
    current_values[i] = current_value
    pid_outputs[i] = pid.compute(current_value)
    current_value += pid_outputs[i] * dt  # Update current value

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time_vals, current_values, label="Current Value")
plt.plot(time_vals, [setpoint]*iterations, label="Setpoint", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("PID Controller Response")
plt.legend()
plt.grid(True)
plt.show()
