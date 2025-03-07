import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===============================
# Bước 1: Định nghĩa dữ liệu cho các mặt cong
# ===============================

# Dữ liệu cho mặt cong thứ nhất (bán kính và chiều cao)
r_values_1 = np.array([11.39517442,
11.36353818,
11.32784181,
11.29468902,
11.25877436,
11.31591799,
11.37189518,
11.42716063,
11.48085363,
11.53516363,
11.58490397,
11.63486141,
11.68417734,
11.73243368,
11.77964346,
11.82581921,
11.87139419,
11.91511645,
11.96035117,
12,
12.04159458,
12.08139065,
12.12105606,
12.15935854,
12.2,
12.23315168,
12.26906679,
12.3032516,
12.33693641,
12.36931688,
12.4016128,
12.43261839,
12.46314567,
12.49199744,
12.52078272,
12.54830666,
12.57656551,
12.6,
12.62537128,
12.64911064,
12.67320007,
12.7,
12.71927671,
12.73891675,
12.75774275,
12.77693234,
12.79531164,
12.81288414,
12.83160161,
12.84562182,
12.86312559,
12.8751699,
12.88875479,
12.90155029,
12.91394595,
12.92439554,
12.93560977,
12.94488316,
12.95414991,
12.96186715,
12.96919427,
12.97574661,
12.98345101,
12.98653148,
12.99576854,
12.99576854,
12.99730741,
12.9988461,
13,
13
])
z_values_1 = np.array([0, 0.1, 0.2, 0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,
3,
3.1,
3.2,
3.3,
3.4,
3.5,
3.6,
3.7,
3.8,
3.9,
4,
4.1,
4.2,
4.3,
4.4,
4.5,
4.6,
4.7,
4.8,
4.9,
5,
5.1,
5.2,
5.3,
5.4,
5.5,
5.6,
5.7,
5.8,
5.9,
6,
6.1,
6.2,
6.3,
6.4,
6.5,
6.6,
6.7,
6.8,
6.9
])

# Dữ liệu cho mặt cong thứ hai (bán kính và chiều cao)
r_values_2 = np.array([22.42074932,
22.4040175,
22.38771985,
22.37073982,
22.35173371,
22.33427859,
22.31636171,
22.29708501,
22.27666941,
22.25758298,
22.23533224,
22.21576017,
22.19481922,
22.173182,
22.15039503,
22.12781056,
22.10452442,
22.06399782,
22.00477221,
21.94401969,
21.88195604,
21.81788257,
21.75247112,
21.68432614,
21.61573501,
21.54553318,
21.47300631,
21.4,
21.32346126,
21.24617613,
21.16530179,
21.0838801,
21.00023809,
20.91243649,
20.82546518,
20.73475343,
20.64218981,
20.54555913,
20.44822731,
20.3482186,
20.24277649,
20.13752716,
20.02823008,
19.91607391,
19.80025252,
19.67968496,
19.55760722,
19.43116054,
19.29818644,
19.1637679,
19.02340663,
18.87670522,
18.72778684,
18.56986807,
18.40679222,
18.23622768,
18.06017719,
17.87344399,
17.67851804,
17.4691156,
17.25253605,
17.01881312,
16.76931722,
16.49848478,
16.20154314,
15.87009767,
15.48967398,
15.03761949,
14.44506836,
13
])
z_values_2 = z_values_1

# Tạo dữ liệu cho góc theta chạy từ 0 đến pi (nửa vòng tròn)
theta = np.linspace(0, np.pi, 200)  # Tăng độ phân giải cho các mặt cong mượt hơn

# Tạo lưới các điểm cho mặt cong thứ nhất và thứ hai
theta_grid_1, z_grid_1 = np.meshgrid(theta, z_values_1)
theta_grid_2, z_grid_2 = np.meshgrid(theta, z_values_2)

# Tính tọa độ x, y cho mặt cong thứ nhất và thứ hai
y_grid_1 = r_values_1[:, np.newaxis] * np.cos(theta_grid_1)
x_grid_1 = r_values_1[:, np.newaxis] * np.sin(theta_grid_1)

y_grid_2 = r_values_2[:, np.newaxis] * np.cos(theta_grid_2)
x_grid_2 = r_values_2[:, np.newaxis] * np.sin(theta_grid_2)

# Định nghĩa giới hạn không gian
x_min, x_max = -23, 23
y_min, y_max = -23, 23
z_min, z_max = 0, 7.0

# ===============================
# Bước 2: Tạo mặt phẳng và các đường cắt
# ===============================

# 2.1: Tạo mặt phẳng z = 1.8, chỉ lấy phần x > 0 và 11.6 <= r <=14.4
x_plane = np.linspace(0, 20, 200)  # Chỉ x từ 0 trở lên
y_plane = np.linspace(-13, 13, 200)  # Y từ -14.4 đến 14.4 để phù hợp với r
X_plane, Y_plane = np.meshgrid(x_plane, y_plane)
Z_plane = np.full_like(X_plane, 6.9)

# Tính bán kính tại mỗi điểm trên mặt phẳng
R_plane = np.sqrt(X_plane**2 + Y_plane**2)

# Tạo mặt nạ cho phần annular (11.6 <= r <=14.4)
mask_annulus = (R_plane >= 13) & (R_plane <= 13)

# Áp dụng mặt nạ: đặt các điểm ngoài vùng annular thành NaN
X_plane_masked = np.where(mask_annulus, X_plane, np.nan)
Y_plane_masked = np.where(mask_annulus, Y_plane, np.nan)
Z_plane_masked = np.where(mask_annulus, Z_plane, np.nan)

# 2.2: Tạo mặt phẳng YZ (x = 0), y >0 và y <0 với các đường cắt

# Đối với mặt phẳng YZ, x =0
# Đối với mỗi mặt cong, khi x =0, theta=0 hoặc pi, y = r*cos(theta)
# Ở theta=0, y=r; ở theta=pi, y=-r

# Tạo đường cắt cho y >0
y_positive_1 = r_values_1 * 1  # theta=0
y_positive_2 = r_values_2 * 1  # theta=0

# Tạo đường cắt cho y <0
y_negative_1 = r_values_1 * (-1)  # theta=pi
y_negative_2 = r_values_2 * (-1)  # theta=pi

# ===============================
# Bước 3: Vẽ các thành phần lên plot
# ===============================

# Khởi tạo plot 3D
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# 3.1: Vẽ mặt phẳng z =1.8 đã cắt
ax.plot_surface(
    X_plane_masked, Y_plane_masked, Z_plane_masked,
    color='green', alpha=0.3, edgecolor='none',
    label='Mặt phẳng z=1.8'
)

# 3.2: Vẽ các đường cắt trên mặt phẳng YZ (x=0)

# Đường cắt cho y >0
ax.plot(
    [0]*len(y_positive_1), y_positive_1, z_values_1,
    color='blue', label='Đường cắt y >0 - Mặt cong 1'
)
ax.plot(
    [0]*len(y_positive_2), y_positive_2, z_values_2,
    color='red', label='Đường cắt y >0 - Mặt cong 2'
)

# Đường cắt cho y <0
ax.plot(
    [0]*len(y_negative_1), y_negative_1, z_values_1,
    color='blue'
)
ax.plot(
    [0]*len(y_negative_2), y_negative_2, z_values_2,
    color='red'
)

# 3.3: Vẽ các mặt cong đã cắt
ax.plot_surface(
    x_grid_1, y_grid_1, z_grid_1,
    color='blue', alpha=0.6, edgecolor='none',
    label='Mặt cong thứ nhất'
)
ax.plot_surface(
    x_grid_2, y_grid_2, z_grid_2,
    color='red', alpha=0.6, edgecolor='none',
    label='Mặt cong thứ hai'
)

# 3.4: Bổ sung thêm hai mặt phẳng kết nối từ z=0 đến z=1.8 cho y>0 và y<0

# 3.4.1: Mặt phẳng cho y >0
# Tạo tham số t từ 0 đến 1 để interpolate giữa hai đường cắt
t = np.linspace(0, 1, 100)
Z_side_positive, T_positive = np.meshgrid(z_values_1, t)

# Interpolate y giữa y_positive_1 và y_positive_2
Y_side_positive = y_positive_1[np.newaxis, :] + T_positive * (y_positive_2 - y_positive_1)[np.newaxis, :]
X_side_positive = np.zeros_like(Y_side_positive)  # x =0

# Vẽ mặt phẳng y >0
ax.plot_surface(
    X_side_positive, Y_side_positive, Z_side_positive,
    color='cyan', alpha=0.3, edgecolor='none',
    label='Mặt phẳng kết nối y >0'
)

# 3.4.2: Mặt phẳng cho y <0
# Tạo tham số t từ 0 đến 1 để interpolate giữa hai đường cắt
Z_side_negative, T_negative = np.meshgrid(z_values_1, t)

# Interpolate y giữa y_negative_1 và y_negative_2
Y_side_negative = y_negative_1[np.newaxis, :] + T_negative * (y_negative_2 - y_negative_1)[np.newaxis, :]
X_side_negative = np.zeros_like(Y_side_negative)  # x =0

# Vẽ mặt phẳng y <0
ax.plot_surface(
    X_side_negative, Y_side_negative, Z_side_negative,
    color='cyan', alpha=0.3, edgecolor='none',
    label='Mặt phẳng kết nối y <0'
)

# 3.5: Bổ sung thêm mặt phẳng vành khuyên tại z=0, x>0, 7 <= r <=19
# Tạo dữ liệu cho vành khuyên
r_new = np.linspace(11.39517442, 22.42074932, 100)
theta_new = np.linspace(0, np.pi, 200)  # theta từ 0 đến pi để đảm bảo x >0
R_new, Theta_new = np.meshgrid(r_new, theta_new)

# Tính tọa độ x, y cho vành khuyên
X_ring = R_new * np.sin(Theta_new)
Y_ring = R_new * np.cos(Theta_new)
Z_ring = np.zeros_like(X_ring)  # z=0

# Vẽ vành khuyên
ax.plot_surface(
    X_ring, Y_ring, Z_ring,
    color='yellow', alpha=0.3, edgecolor='none',
    label='Vành khuyên (7 <= r <=19, x >0, z=0)'
)

# ===============================
# Bước 4: Thiết lập nhãn trục và hiển thị plot
# ===============================

# Thiết lập nhãn trục
ax.set_xlabel('X (cm)', fontsize=12)
ax.set_ylabel('Y (cm)', fontsize=12)
ax.set_zlabel('Z (cm)', fontsize=12)
ax.set_title('Khối 3D Workspace của Robot khi gắp vuông góc với mặt sàn', fontsize=14)

# Thiết lập giới hạn trục để rõ ràng hơn
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim(z_min, z_max)

# Thêm legend để giải thích các thành phần
# Vì plot_surface không hỗ trợ label trực tiếp, nên cần tạo proxy artists để thêm vào legend
from matplotlib.patches import Patch
# legend_elements = [
#     Patch(facecolor='green', edgecolor='none', alpha=0.3, label='Mặt phẳng z=1.8'),
#     Patch(facecolor='blue', edgecolor='none', alpha=0.6, label='Mặt cong thứ nhất'),
#     Patch(facecolor='red', edgecolor='none', alpha=0.6, label='Mặt cong thứ hai'),
#     Patch(facecolor='cyan', edgecolor='none', alpha=0.3, label='Mặt phẳng kết nối y>0 và y<0'),
#     Patch(facecolor='yellow', edgecolor='none', alpha=0.3, label='Vành khuyên (7 <= r <=19, x >0, z=0)')
# ]
# ax.legend(handles=legend_elements, loc='upper right')

# Hiển thị plot
plt.show()

