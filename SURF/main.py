import cv2
import numpy as np

# Đọc ảnh mẫu và ảnh cần kiểm tra
img1 = cv2.imread("temp.jpg", cv2.IMREAD_GRAYSCALE)  # Ảnh mẫu
img2 = cv2.imread("case2.jpg", cv2.IMREAD_GRAYSCALE)  # Ảnh cần kiểm tra

# Khởi tạo ORB detector
orb = cv2.ORB_create(nfeatures=1000)

# Tính toán keypoints và descriptors cho cả hai ảnh
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# Khởi tạo bộ so khớp BFMatcher với Hamming Distance
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# So khớp các descriptors giữa hai ảnh
matches = bf.knnMatch(des1, des2, k=2)

# Lọc các khớp tốt bằng thuật toán Lowe's ratio test
good_matches = []
for m, n in matches:
    if m.distance < 0.75 * n.distance:
        good_matches.append(m)

# Tính Homography nếu có đủ khớp tốt
MIN_MATCH_COUNT = 4  # Số khớp tốt tối thiểu để tính Homography
if len(good_matches) >= MIN_MATCH_COUNT:
    # Lấy các điểm tương ứng
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    # Tính Homography để tìm đối tượng trong khung hình
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matches_mask = mask.ravel().tolist()

    # Tìm các góc của vật mẫu
    h, w = img1.shape
    obj_corners = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)

    # Chuyển đổi các góc của vật mẫu vào ảnh cần kiểm tra
    scene_corners = cv2.perspectiveTransform(obj_corners, M)

    # Vẽ khung bao quanh vật trong ảnh cần kiểm tra
    img2 = cv2.polylines(img2, [np.int32(scene_corners)], True, (0, 255, 0), 3, cv2.LINE_AA)

    # Đếm số đối tượng tương tự
    num_objects = 1  # Đối tượng đầu tiên đã xác định
else:
    print("Không đủ khớp tốt để tìm vật mẫu.")
    num_objects = 0

# Hiển thị số lượng đối tượng
print(f"Số lượng vật mẫu được phát hiện: {num_objects}")

# Hiển thị ảnh với các đối tượng được phát hiện
cv2.imshow("Detected Objects", img2)
cv2.waitKey(0)
cv2.destroyAllWindows()
