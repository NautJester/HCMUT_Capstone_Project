import cv2
import numpy as np
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

    _,binary = cv2.threshold(stretched_img, 75, 255, cv2.THRESH_BINARY)

    return binary


# Đọc ảnh mẫu và ảnh cần tìm
img = cv2.imread('temp7.jpg', 0)

processed_img = process_image(img)

cv2.imshow('Processed', processed_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
