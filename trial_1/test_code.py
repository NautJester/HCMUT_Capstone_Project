import numpy as np
import cv2


def drawBox(img, keypoints, matches):
    # Vẽ các keypoints trên ảnh
    for match in matches:
        # Vẽ điểm khớp giữa các khung hình
        img1_idx = match.queryIdx
        img2_idx = match.trainIdx
        x1, y1 = keypoints[0][img1_idx].pt
        x2, y2 = keypoints[1][img2_idx].pt
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

    cv2.putText(img, "Tracking", (50, 80), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 1)


def increase_contrast(image):
    # Chuyển đổi sang không gian màu YUV
    yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

    # Tăng độ tương phản bằng cách điều chỉnh kênh Y
    yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])

    # Chuyển đổi lại về không gian màu BGR
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)


def find_contours(image):
    # Chuyển đổi sang ảnh xám
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Áp dụng ngưỡng để phát hiện các đường viền
    _, thresh = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

    # Tìm các đường viền
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours


def detect_and_compute(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create()
    keypoints, descriptors = orb.detectAndCompute(gray_image, None)
    return keypoints, descriptors


def match_features(descriptors1, descriptors2):
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)
    return matches


def select_roi(cap):
    success, img = cap.read()
    if not success:
        print("Không thể đọc hình ảnh từ camera.")
        return None

    # Cho phép người dùng chọn ROI
    bbox = cv2.selectROI("Select ROI", img, False)
    cv2.destroyWindow("Select ROI")
    return img[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])], bbox


camera_url = "http://192.168.1.4:4747/video"  # Thay x.x bằng địa chỉ IP của điện thoại
cap = cv2.VideoCapture(camera_url)

# Chọn ROI từ video
reference_image, bbox = select_roi(cap)
if reference_image is None:
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Tăng độ tương phản cho ROI
reference_image = increase_contrast(reference_image)

# Tìm các đường viền trong ROI
contours = find_contours(reference_image)

# Chọn đường viền đầu tiên (nếu có) để dùng cho ORB
if contours:
    # Vẽ đường viền lên ảnh tham chiếu
    cv2.drawContours(reference_image, contours, -1, (0, 255, 0), 3)
    reference_image = cv2.boundingRect(contours[0])  # Lấy đường bao của đường viền đầu tiên
    reference_keypoints, reference_descriptors = detect_and_compute(reference_image)
else:
    print("Không tìm thấy đường viền nào trong ROI.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

while True:
    timer = cv2.getTickCount()
    success, img = cap.read()
    if not success:
        break

    # Tăng độ tương phản cho khung hình hiện tại
    img_contrast = increase_contrast(img)

    # Tìm các đường viền trong khung hình hiện tại
    contours_current = find_contours(img_contrast)

    # Nếu có đường viền, tiến hành ORB
    if contours_current:
        # Vẽ đường viền lên khung hình hiện tại
        cv2.drawContours(img_contrast, contours_current, -1, (0, 255, 0), 3)

        # Khớp các đặc trưng giữa ảnh hiện tại và ảnh tham chiếu
        current_keypoints, current_descriptors = detect_and_compute(img_contrast)
        matches = match_features(reference_descriptors, current_descriptors)

        # Vẽ bounding box cho đối tượng được theo dõi
        drawBox(img, (reference_keypoints, current_keypoints), matches)

    cv2.imshow("Tracking", img)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
