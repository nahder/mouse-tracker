import cv2
import numpy as np
import pyrealsense2 as rs


def get_corners():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    corner_points = []
    should_stop = False

    def draw_lines(image, points):
        if len(points) == 4:
            for i in range(4):
                cv2.line(image, points[i], points[(i + 1) % 4], (0, 255, 0), 2)

    def close_windows():
        cv2.destroyAllWindows()
        pipeline.stop()

    def done(*args):
        nonlocal should_stop
        if len(corner_points) == 4:
            print("Done with corner points:", corner_points)
            should_stop = True
        else:
            print("Please select exactly 4 points")

    def redo(*args):
        nonlocal corner_points
        corner_points = []
        print("Redo clicked. Resetting points.")

    def mouse_callback(event, x, y, flags, param):
        nonlocal corner_points
        if event == cv2.EVENT_MBUTTONDOWN:
            if len(corner_points) < 4:
                corner_points.append((x, y))
                print(f"Selected corner: {x, y}")

    cv2.namedWindow("Select Corners")
    cv2.setMouseCallback("Select Corners", mouse_callback)

    cv2.createButton("Redo", redo, None, cv2.QT_PUSH_BUTTON, 1)
    cv2.createButton("Done", done, None, cv2.QT_PUSH_BUTTON, 1)

    while not should_stop:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        color_image = np.rot90(color_image, 3)
        color_image = cv2.flip(color_image, 1)
        color_image = cv2.flip(color_image, 0)

        #TODO; fix crop
        color_image = color_image[30:480, 10:450]

        display_image = color_image.copy()

        for point in corner_points:
            cv2.circle(display_image, point, 5, (0, 255, 0), -1)

        draw_lines(display_image, corner_points)

        cv2.imshow("Select Corners", display_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            should_stop = True

    close_windows()

    print("Corner Points:", corner_points)

    return corner_points


def calculate_homography(corner_points, ppi):
    real_world_size = 6  #

    pixel_size = real_world_size * ppi

    dst_points = np.array(
        [
            [0, 0],  # top-left corner
            [pixel_size, 0],  # top-right corner
            [pixel_size, pixel_size],  # bottom-right corner
            [0, pixel_size],  # bottom-left corner
        ],
        dtype=np.float32,
    )

    src_points = np.array(corner_points, dtype=np.float32)

    H, _ = cv2.findHomography(src_points, dst_points)

    return H, int(pixel_size), int(pixel_size)


if __name__ == "__main__":
    corners = get_corners()
    print("Corners returned:", corners)

    if len(corners) == 4:
        ppi = 45

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            color_image = np.rot90(color_image, 3)
            color_image = cv2.flip(color_image, 1)
            color_image = cv2.flip(color_image, 0)

            color_image = color_image[30:480, 10:450]

            H, new_width, new_height = calculate_homography(corners, ppi)

            corrected_image = cv2.warpPerspective(
                color_image, H, (new_width, new_height)
            )

            display_image = color_image.copy()
            for point in corners:
                cv2.circle(display_image, point, 5, (0, 255, 0), -1)
            for i in range(4):
                cv2.line(
                    display_image, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2
                )

            corrected_image_resized = cv2.resize(
                corrected_image, (color_image.shape[1], color_image.shape[0])
            )

            combined_image = np.hstack((display_image, corrected_image_resized))
            cv2.imshow("Original with Corners and Corrected View", combined_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()
        pipeline.stop()
