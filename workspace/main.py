import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

# Giả sử dữ liệu tìm được:
data = [
    {'z': 3.5, 'd_min': 5.0, 'd_max': 15.0},
    {'z': 4.0, 'd_min': 6.0, 'd_max': 16.0},
    {'z': 4.5, 'd_min': 7.0, 'd_max': 17.0}
]

# Vẽ không gian làm việc
fig, ax = plt.subplots(figsize=(8, 8))

# Vẽ hình vành khuyên cho từng giá trị z
for entry in data:
    z = entry['z']
    d_min = entry['d_min']
    d_max = entry['d_max']

    # Tạo vành khuyên bằng cách vẽ hai đường tròn
    circle_outer = Circle((0, 0), d_max, color='blue', alpha=0.3, label=f"z = {z}" if z == data[0]['z'] else "")
    circle_inner = Circle((0, 0), d_min, color='white', alpha=1.0)

    # Thêm các đường tròn vào trục
    ax.add_artist(circle_outer)
    ax.add_artist(circle_inner)

# Cấu hình trục
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_aspect('equal', adjustable='datalim')
ax.set_title("Không gian làm việc")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.legend(loc='upper right')

# Hiển thị hình
plt.show()

