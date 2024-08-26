import threading
import tensorflow as tf
import serial
from dlclive import DLCLive, Processor
from corner_clicker import get_corners, calculate_homography
import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque

gpu_devices = tf.config.experimental.list_physical_devices("GPU")
for device in gpu_devices:
    tf.config.experimental.set_memory_growth(device, True)

class LiveTracker:

    def __init__(self, arduino, corners, model_path, filter_type="moving_average", record=False, output_path="output.avi", enable_display=True, scale_factor=0.5,):
        self.corners = corners
        self.arduino = arduino
        self.filter_type = filter_type
        self.window_size = 2
        self.pose_history = deque(maxlen=self.window_size)

        self.dlc_processor = Processor()

        self.model_path = model_path
        self.dlc_live = DLCLive(
            model_path=self.model_path,
            processor=self.dlc_processor,
        )

        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(self.cfg)

        self.pose = []
        self.frame = None
        self.new_width = None
        self.new_height = None
        self.pose_lock = threading.Lock()
        self.frame_lock = threading.Lock()

        self.record = record
        self.out = None
        self.video_initialized = False
        self.output_path = output_path
        self.enable_display = enable_display
        self.scale_factor = scale_factor  

        # scale the corners based on the scale factor
        self.scaled_corners = [(int(x * self.scale_factor), int(y * self.scale_factor)) for (x, y) in self.corners]

        if filter_type == "kalman":
            self.kf = cv2.KalmanFilter(4, 2)
            self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
            self.kf.processNoiseCov = np.identity(4, dtype=np.float32) * 0.1  
            self.kf.measurementNoiseCov = np.identity(2, dtype=np.float32) * 0.05  
            self.kf.statePost = np.array([0, 0, 0, 0], dtype=np.float32)

    def get_frame(self):
        frames = self.pipe.wait_for_frames()

        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # rotate and flip the image
        color_image = np.rot90(color_image, 3)
        color_image = cv2.flip(color_image, 1)
        color_image = cv2.flip(color_image, 0)
        color_image = color_image[30:480, 10:450]

        # scale down the resolution
        color_image = cv2.resize(color_image, None, fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_AREA)

        # apply the homography matrix
        H, self.new_width, self.new_height = calculate_homography(
            corner_points=self.scaled_corners, ppi=45  # Use scaled corners
        )

        corrected_image = cv2.warpPerspective(color_image, H, (self.new_width, self.new_height))

        if self.record and not self.video_initialized:
            fourcc = cv2.VideoWriter_fourcc("M", "J", "P", "G")
            self.out = cv2.VideoWriter(
                self.output_path, fourcc, 30, (self.new_width, self.new_height)
            )
            self.video_initialized = True

        return corrected_image

    def apply_kalman_filter(self, pose):
        prediction = self.kf.predict()
        measurement = np.array([[pose[0]], [pose[1]]], dtype=np.float32)
        self.kf.correct(measurement)
        return self.kf.statePost[:2]

    def apply_moving_average_filter(self, pose):
        self.pose_history.append(pose)
        return np.mean(self.pose_history, axis=0)

    def update_pose(self):
        while True:
            frame = self.get_frame()
            pose = self.dlc_live.get_pose(frame)

            #NOTE: The pose is a list of numpy arrays. depending on the number of body parts detected, the length of the list will vary
            pose = pose[0]

            if self.filter_type == "kalman":
                pose = self.apply_kalman_filter(pose)
            elif self.filter_type == "moving_average":
                pose = self.apply_moving_average_filter(pose)

            with self.pose_lock:
                self.pose = pose

            with self.frame_lock:
                self.frame = frame

            if self.record and self.out is not None:
                self.out.write(frame)

            minX, maxX = 350, 4125  
            minY, maxY = 180, 3950  

            x_steps = int((pose[0] / self.new_width) * (maxX - minX) + minX)
            y_steps = int((pose[1] / self.new_height) * (maxY - minY) + minY)

            # send to arduino
            self.arduino.write(f"{x_steps} {y_steps}\n".encode())

    def display(self):
        try:
            while True:
                with self.frame_lock:
                    if self.frame is not None:
                        frame = self.frame.copy()
                    else:
                        continue

                with self.pose_lock:
                    if self.pose.size > 0:
                        cv2.circle(
                            frame,
                            (int(self.pose[0]), int(self.pose[1])),
                            5,
                            (0, 0, 255),
                            -1,
                        )

                if len(self.corners) == 2:
                    top_left = self.corners[0]
                    bottom_right = self.corners[1]
                    cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

                cv2.imshow("Live Tracker", frame)

                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cv2.destroyAllWindows()
            if self.record and self.out is not None:
                self.out.release()
            self.pipe.stop()

    def stop(self):
        # Clean up resources even if display is not called
        if self.record and self.out is not None:
            self.out.release()
        self.pipe.stop()


def main():
    # Initialize the Arduino serial connection
    arduino = serial.Serial(port="/dev/ttyACM0", baudrate=2000000, timeout=1)

    # Detect the corners of the chamber frame
    corners = get_corners()

    # Choose filter type directly in the tracker initialization
    filter_type = "moving_average" 

    input("Corners detected. Press Enter to start tracking...")

    tracker = LiveTracker(
        arduino, corners, model_path="/home/nader/Desktop/track_finger/DLC_tracking_finger_mobilenet_v2_0.75_iteration-0_shuffle-1",
        filter_type=filter_type, record=False, enable_display=True, scale_factor=0.3  
    )

    tracker.dlc_live.init_inference(tracker.get_frame())
    pose_thread = threading.Thread(target=tracker.update_pose)
    pose_thread.daemon = True
    pose_thread.start()

    if tracker.enable_display:
        tracker.display()
    else:
        # If display is not enabled, wait for the user to stop the program manually
        input("Tracking without display. Press Enter to stop...")

    tracker.stop()


if __name__ == "__main__":
    main()
