import cv2
import numpy as np

# Hàm để kiểm tra circularity
def is_circle(contour, threshold=0.85):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    circularity = (4 * np.pi * area) / (perimeter ** 2)
    return circularity >= threshold
def contrast_stretching(image, y):
    # Chuyển đổi ảnh sang định dạng float32
    image_float = image.astype(np.float32)

    # Giãn cách độ tương phản theo công thức đã cho
    g = (image_float / 255) ** y * 255

    # Giới hạn giá trị và chuyển đổi lại về uint8
    g = np.clip(g, 0, 255).astype(np.uint8)

    return g
def process_image(frame):

    # Áp dụng Box Filter
    box_blur = cv2.boxFilter(frame, ddepth=-1, ksize=(5, 5), normalize=True)
    # Tăng sáng
    bright = cv2.convertScaleAbs(box_blur, alpha=2.0, beta=30)

    # Áp dụng giãn cách độ tương phản
    stretched_img = contrast_stretching(box_blur, 1)

    _,binary = cv2.threshold(stretched_img, 140, 255, cv2.THRESH_BINARY)

    return binary

img = cv2.imread('temp7.jpg',1)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

binary_img = process_image(gray)

contours, hierachy = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


# Danh sách để lưu lại các contours là hình tròn
circle_contours = []

for contour in contours:
    if is_circle(contour):
        circle_contours.append(contour)


# Tính trung bình tọa độ x và y
x_coords = [point[0][0] for point in circle_contours[0]]
y_coords = [point[0][1] for point in circle_contours[0]]

print(x_coords)

x_center = sum(x_coords) / len(x_coords)
y_center = sum(y_coords) / len(y_coords)

center = (int(x_center), int(y_center))
distances = [np.sqrt((x - x_center) ** 2 + (y - y_center) ** 2) for x, y in zip(x_coords, y_coords)]
radius = sum(distances) / len(distances)
print(f"Tọa độ trọng tâm (centroid): {center}")
print(f"Bán kính: {radius}")

# Vẽ các contour hình tròn lên ảnh gốc
cv2.drawContours(img, circle_contours, -1, (0, 255, 0), 2)
cv2.circle(img, center, 3, (0, 0, 255), -1)  # Vẽ trọng tâm màu đỏ
# print(circle_contours[0])
print(type(circle_contours))  # Kết quả nên là <class 'numpy.ndarray'>

# Hiển thị ảnh có contour
cv2.imshow("Contours", img)
cv2.waitKey(0)
cv2.destroyAllWindows()


