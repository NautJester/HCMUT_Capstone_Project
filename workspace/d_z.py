import numpy as np
import pandas as pd

# Define constants
d1, a2, a3, a4, d5 = 14.4, 10.5, 13.0, 7.0, 11.0
PI = np.pi

# Function to convert radians to degrees
def rad_to_deg(radian):
    return np.degrees(radian)

# Inverse kinematics calculation
def inverse_kinematics(object_pos):
    x, y, z = object_pos[0], object_pos[1], object_pos[2]
    zc = z + (a4 + d5)
    # Compute theta1
    theta1 = np.arctan2(y, x)

    # Compute r and s
    r = np.hypot(x, y)
    s = zc - d1

    # Compute theta2 and theta3
    D = (r ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (-2 * a2 * a3)

    if D < -1 or D > 1:
        return None  # Unreachable target

    theta3 = np.arctan2(np.sqrt(1 - D ** 2), D)

    theta2 = PI - np.arctan2(s, r) - np.arctan2(a3 * np.sin(PI - theta3), a2 + a3 * np.cos(PI - theta3))

    theta4 = PI - (2 * PI - PI / 2 - theta3 - PI + theta2 - PI / 2)

    theta5 = theta1  # Assuming no rotation in 5th axis

    # Convert angles to degrees
    thetas_deg = rad_to_deg([theta1, theta2, theta3, theta4, theta5])

    # Filter all angles based on new constraints
    if (0 <= thetas_deg[0] <= 180 and
        90 <= thetas_deg[1] <= 180 and
        60 <= thetas_deg[2] <= 150 and
        0 <= thetas_deg[3] <= 180 and
        0 <= thetas_deg[4] <= 180):
        return thetas_deg
    else:
        return None

# Create a list of object positions
object_positions = [(x, y, z) for x in np.arange(0, 22.1, 0.1) for y in np.arange(0, 22.1, 0.1) for z in np.arange(0, 10.0, 0.1)]

# Calculate angles for each object position
results = []
for obj_pos in object_positions:
    thetas = inverse_kinematics(obj_pos)
    if thetas is not None:
        d = np.hypot(obj_pos[0], obj_pos[1])  # Calculate d = sqrt(x^2 + y^2)
        results.append([*obj_pos, d, *thetas])

# Create DataFrame
columns = ['x', 'y', 'z', 'd', 'theta1 (deg)', 'theta2 (deg)', 'theta3 (deg)', 'theta4 (deg)', 'theta5 (deg)']
df = pd.DataFrame(results, columns=columns)

# Filter results for max and min d for each z
max_d_df = df.loc[df.groupby('z')['d'].idxmax()][['x', 'y', 'z', 'd', 'theta1 (deg)', 'theta2 (deg)', 'theta3 (deg)', 'theta4 (deg)', 'theta5 (deg)']]
max_d_df = max_d_df.rename(columns={'d': 'd_max'})  # Rename column to d_max

min_d_df = df.loc[df.groupby('z')['d'].idxmin()][['x', 'y', 'z', 'd', 'theta1 (deg)', 'theta2 (deg)', 'theta3 (deg)', 'theta4 (deg)', 'theta5 (deg)']]
min_d_df = min_d_df.rename(columns={'d': 'd_min'})  # Rename column to d_min

# Merge max_d_df and min_d_df on 'z'
filtered_df = pd.merge(max_d_df, min_d_df, on='z', how='outer')

# Save to Excel
output_path = r'C:\Users\TUAN\Capstone Project\workspace\vertical_grasp_5_filtered.xlsx'
filtered_df.to_excel(output_path, index=False)

print(f"Filtered results saved to {output_path}")


