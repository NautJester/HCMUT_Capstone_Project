import numpy as np
import matplotlib.pyplot as plt

# Định nghĩa các hằng số
PWM_FREQUENCY = 50
MAX_RESOLUTION = 4095
MIN_PULSE_WIDTH = 102.375  # 1ms
MAX_PULSE_WIDTH = 511.875  # 2ms
PWM_PERIOD_MS = 20.0  # 20ms
DT = 0.01  # 20ms = 50Hz


# Hàm chuyển đổi góc sang PWM
def angleToPWM(angle):
    pwm_value = MIN_PULSE_WIDTH + (angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 180.0
    return pwm_value


# Hàm chuyển đổi PWM sang góc
def PWMToAngle(pwm_value):
    angle = (pwm_value - MIN_PULSE_WIDTH) * 180.0 / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    return angle


# Class PID Controller
class PID_Controller:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output


# Hàm khởi tạo LSPB
class LSPB_Params:
    def __init__(self, q0, qf, v_max, a_max, t_total):
        self.q0 = q0
        self.qf = qf
        self.v_max = v_max
        self.t_total = t_total
        self.a_max = a_max

        self.t_acc = v_max / self.a_max
        self.t_const = t_total - 2 * self.t_acc

    def calculate_position(self, t):
        if self.qf > self.q0:  # Trường hợp qf > q0
            if t < self.t_acc:
                return self.q0 + 0.5 * self.a_max * t * t
            elif t < (self.t_acc + self.t_const):
                return (self.q0 +
                        0.5 * self.a_max * self.t_acc * self.t_acc +
                        self.v_max * (t - self.t_acc))
            elif t < self.t_total:
                t_dec = t - self.t_acc - self.t_const
                return (self.q0 +
                        0.5 * self.a_max * self.t_acc * self.t_acc +
                        self.v_max * self.t_const +
                        self.v_max * t_dec -
                        0.5 * self.a_max * t_dec * t_dec)
            else:
                return self.qf
        elif self.qf < self.q0:  # Trường hợp qf < q0
            if t < self.t_acc:
                return self.q0 - 0.5 * self.a_max * t * t
            elif t < (self.t_acc + self.t_const):
                return (self.q0 -
                        0.5 * self.a_max * self.t_acc * self.t_acc -
                        self.v_max * (t - self.t_acc))
            elif t < self.t_total:
                t_dec = t - self.t_acc - self.t_const
                return (self.q0 -
                        0.5 * self.a_max * self.t_acc * self.t_acc -
                        self.v_max * self.t_const -
                        self.v_max * t_dec +
                        0.5 * self.a_max * t_dec * t_dec)
            else:
                return self.qf
        else:  # Trường hợp qf == q0
            return self.qf


# Khởi tạo các giá trị
current_angles = [90.0, 90.0, 90.0, 90.0, 0.0]
target_angles = [0.0, 100.0, 101.0, 0.0, 90.0]
lspb = [LSPB_Params(current_angles[i], target_angles[i], 60, 120, 2.0) for i in range(5)]
angles = [0.0] * 5

# Tạo danh sách để lưu trữ góc theo thời gian
time_values = []
angle_values_1 = []  # Lưu trữ giá trị góc cho khớp thứ 2

# Vòng lặp điều khiển cho từng khớp
for i in range(4, -1, -1):
    current_pwm = angleToPWM(current_angles[i])
    pid = PID_Controller(0.1, 0.03, 0, current_pwm, 0.01)  # khởi tạo PID với current_pwm
    for t in np.arange(0, lspb[i].t_total + DT, DT):
        target_angle = lspb[i].calculate_position(t)
        target_pwm = angleToPWM(target_angle)
        pid.setpoint = target_pwm  # Cập nhật setpoint cho PID

        # Điều khiển servo để đạt tới góc mục tiêu bằng PID
        while abs(current_pwm - target_pwm) > 0.1:
            current_pwm += pid.compute(current_pwm) * 0.01
            angles[i] = PWMToAngle(current_pwm)  # Chuyển đổi PWM sang góc

            # Lưu giá trị thời gian và góc cho khớp thứ 2
            if i == 4:  # Nếu là khớp thứ 2 (angles[1])
                time_values.append(t)
                angle_values_1.append(angles[i])

        # Lưu góc cuối cùng tại thời điểm t
        angles[i] = PWMToAngle(current_pwm)
        if i == 4:  # Nếu là khớp thứ 2 (angles[1])
            time_values.append(t)
            angle_values_1.append(angles[i])

# Kết quả
print("Final Angles:", angles)

# Vẽ đồ thị cho góc khớp thứ 2
plt.plot(time_values, angle_values_1, label='Joint 2', color='blue')

plt.title('Joint 2 Angle over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.grid()
plt.show()
