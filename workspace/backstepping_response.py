import numpy as np
import matplotlib.pyplot as plt

# Hàm khởi tạo LSPB
def LSPB_Init(q0, qf, v_max, a_max, t_total):
    t_acc = v_max / a_max
    t_const = t_total - 2 * t_acc
    return q0, qf, t_acc, t_const, t_total, v_max, a_max

# Tính toán vị trí dựa trên LSPB
def LSPB_CalculatePosition(q0, qf, t_acc, t_const, t_total, v_max, a_max, t):
    if qf - q0 > 0:
        if t < t_acc:
            q = q0 + 0.5 * a_max * t ** 2
        elif t < (t_acc + t_const):
            q = q0 + 0.5 * a_max * t_acc ** 2 + v_max * (t - t_acc)
        elif t < t_total:
            t_dec = t - t_acc - t_const
            q = q0 + 0.5 * a_max * t_acc ** 2 + v_max * t_const + \
                v_max * t_dec - 0.5 * a_max * t_dec ** 2
        else:
            q = qf
    elif qf - q0 < 0:
        if t < t_acc:
            q = q0 - 0.5 * a_max * t ** 2
        elif t < (t_acc + t_const):
            q = q0 - 0.5 * a_max * t_acc ** 2 - v_max * (t - t_acc)
        elif t < t_total:
            t_dec = t - t_acc - t_const
            q = q0 - 0.5 * a_max * t_acc ** 2 - v_max * t_const - \
                v_max * t_dec + 0.5 * a_max * t_dec ** 2
        else:
            q = qf
    else:
        q = qf
    return q

# Tính toán vận tốc
def LSPB_CalculateVelocity(t_acc, t_const, t_total, v_max, a_max, t):
    if t < t_acc:
        return a_max * t
    elif t < (t_acc + t_const):
        return v_max
    elif t < t_total:
        t_dec = t - t_acc - t_const
        return v_max - a_max * t_dec
    else:
        return 0

# Phương pháp Backstepping
def Backstepping(q, v, dt, current_q, current_v):
    # Thông số điều khiển
    Kp = 1.0  # Hệ số tỷ lệ
    Kd = 0.5  # Hệ số vi phân

    # Tính toán e1 và e2
    e1 = q - current_q  # Lỗi vị trí
    e2 = v - current_v  # Lỗi vận tốc

    # Tín hiệu điều khiển từ phương pháp Backstepping
    u = Kp * e1 + Kd * e2  # Tín hiệu điều khiển

    # Cập nhật vận tốc và vị trí
    delta_v = u * dt
    delta_q = delta_v * dt  # Tích lũy vận tốc để cập nhật vị trí

    return delta_v, delta_q


# Các thông số
q0 = 102.375           # Vị trí bắt đầu
qf = 307         # Vị trí kết thúc
t_total = 1     # Thời gian tổng
v_max = (qf - q0) / t_total      # Vận tốc tối đa
a_max = 400      # Gia tốc tối đa
dt = 0.1        # Thời gian mẫu
DT = 0.1
# Khởi tạo các tham số
t_acc = v_max / a_max
t_const = t_total - 2 * t_acc

# Gọi hàm Backstepping

positions = []
velocities = []
t = 0
current_velocity = 0
current_angle = q0
while (t <= t_total + dt):
    target_angle = LSPB_CalculatePosition(q0, qf, t_acc, t_const, t_total, v_max, a_max, t)
    target_velocity = LSPB_CalculateVelocity(t_acc, t_const, t_total, v_max, a_max, t)
    # print(target_angle)
    # print(target_velocity)
    while (abs(current_angle - target_angle) > 0.1):
        delta_v, delta_q = Backstepping(target_angle, target_velocity, dt, current_angle, current_velocity)
        current_velocity += delta_v
        current_angle += delta_q

        print(current_angle)
        positions.append(current_angle)
        velocities.append(current_velocity)
    t += dt


# Danh sách thời gian, vị trí và vận tốc
t_values = np.arange(0, t_total + dt, dt)
# Vẽ đồ thị
plt.figure(figsize=(12, 10))

# Vẽ vị trí
plt.subplot(2, 1, 1)
plt.plot(t_values, positions, label='Position (q)', color='b')
plt.title('LSPB Position over Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (q)')
plt.grid()
plt.legend()

# Vẽ vận tốc
plt.subplot(2, 1, 2)
plt.plot(t_values, velocities, label='Velocity (v)', color='r')
plt.title('LSPB Velocity over Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (v)')
plt.grid()
plt.legend()



plt.tight_layout()
plt.show()

