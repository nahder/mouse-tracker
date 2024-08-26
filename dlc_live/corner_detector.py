import cv2
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

contrast = 3
brightness = 78
blur_ksize = 1
block_size = 4
C = 0
dilation_size = 1
erosion_size = 1
min_contour_area = 1000


def nothing(x):
    pass


def increase_contrast(image, alpha, beta):
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)


def process_image():
    global contrast, brightness, blur_ksize, block_size, C, dilation_size, erosion_size, min_contour_area

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    color_image = color_image[81:335, 210:468]

    alpha = contrast / 10.0
    beta = brightness - 50
    enhanced_image = increase_contrast(color_image, alpha, beta)

    gray_image = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2GRAY)

    ksize = blur_ksize if blur_ksize % 2 == 1 else blur_ksize + 1  # ensure ksize is odd
    blurred_image = cv2.medianBlur(gray_image, ksize)


    blockSize = (
        block_size if block_size % 2 == 1 else block_size + 1
    )  # ensure blockSize is odd
    blockSize = max(3, blockSize)  
    binary_image = cv2.adaptiveThreshold(
        blurred_image,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        blockSize,
        C,
    )

    dilation_kernel_size = (
        dilation_size if dilation_size % 2 == 1 else dilation_size + 1,
        dilation_size if dilation_size % 2 == 1 else dilation_size + 1,
    )
    erosion_kernel_size = (
        erosion_size if erosion_size % 2 == 1 else erosion_size + 1,
        erosion_size if erosion_size % 2 == 1 else erosion_size + 1,
    )
    erosion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, erosion_kernel_size)
    dilation_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, dilation_kernel_size)

    morph_image = cv2.dilate(binary_image, dilation_kernel, iterations=1)
    morph_image = cv2.erode(morph_image, erosion_kernel, iterations=1)

    contours, _ = cv2.findContours(
        morph_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    morph_image_with_contours = cv2.cvtColor(morph_image, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(morph_image_with_contours, contours, -1, (0, 255, 0), 2)

    if contours:
        filtered_contours = [
            cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area
        ]
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)

            # approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)

            if (
                len(approx) == 4
            ):  # if the approximated contour has 4 points, it's likely a square
                for point in approx:
                    x, y = point.ravel()
                    cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
                cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 2)

    combined_image = np.hstack((color_image, morph_image_with_contours))

    cv2.imshow("Processed Image", combined_image)


cv2.namedWindow("Processed Image")

cv2.createTrackbar("Contrast", "Processed Image", contrast, 30, nothing)
cv2.createTrackbar("Brightness", "Processed Image", brightness, 100, nothing)
cv2.createTrackbar("Blur", "Processed Image", blur_ksize, 20, nothing)
cv2.createTrackbar("Block Size", "Processed Image", block_size, 50, nothing)
cv2.createTrackbar("C", "Processed Image", C, 10, nothing)
cv2.createTrackbar("Dilation Size", "Processed Image", dilation_size, 20, nothing)
cv2.createTrackbar("Erosion Size", "Processed Image", erosion_size, 20, nothing)
cv2.createTrackbar(
    "Min Contour Area", "Processed Image", min_contour_area, 10000, nothing
)

while True:
    contrast = cv2.getTrackbarPos("Contrast", "Processed Image")
    brightness = cv2.getTrackbarPos("Brightness", "Processed Image")
    blur_ksize = cv2.getTrackbarPos("Blur", "Processed Image")
    block_size = cv2.getTrackbarPos("Block Size", "Processed Image")
    C = cv2.getTrackbarPos("C", "Processed Image")
    dilation_size = cv2.getTrackbarPos("Dilation Size", "Processed Image")
    erosion_size = cv2.getTrackbarPos("Erosion Size", "Processed Image")
    min_contour_area = cv2.getTrackbarPos("Min Contour Area", "Processed Image")

    process_image()

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

pipeline.stop()
cv2.destroyAllWindows()
