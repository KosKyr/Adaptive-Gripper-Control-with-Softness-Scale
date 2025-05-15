### File: inputFromCamera.py
import cv2
import numpy as np
import os
import time
#import pyrealsense2 as rs

class InputFromCamera:
    def __init__(self, use_webcam=True, dataset_path=None, use_rgbd=False):
        self.use_webcam = use_webcam
        self.dataset_path = dataset_path
        self.use_rgbd = use_rgbd

        if self.use_webcam:
            print("Initializing webcam...")
            self.cap = cv2.VideoCapture(0)
            time.sleep(2)
            self.cap.set(3, 640)
            self.cap.set(4, 480)
            if not self.cap.isOpened():
                raise RuntimeError("❌ Webcam could not be opened.")
            else:
                print("✅ Webcam opened successfully.")
        else:
            if not os.path.exists(self.dataset_path):
                raise ValueError(f"Dataset path '{self.dataset_path}' does not exist.")
            self.rgb_files = sorted([f for f in os.listdir(self.dataset_path) if f.endswith('-color.png')])
            self.depth_files = sorted([f for f in os.listdir(self.dataset_path) if f.endswith('-depth.png')])
            self.dataset_idx = 0

    def get_frame(self):
        if self.use_webcam:
            success, rgb_frame = self.cap.read()
            if not success or rgb_frame is None:
                raise RuntimeError("❌ Error: Could not read frame from webcam.")

            if self.use_rgbd:
                return
                # if not hasattr(self, 'realsense_pipeline'):
                #     self.realsense_pipeline = rs.pipeline()
                #     config = rs.config()
                #     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                #     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                #     self.realsense_pipeline.start(config)
                #     self.realsense_align = rs.align(rs.stream.color)

                # frames = self.realsense_pipeline.wait_for_frames()
                # aligned_frames = self.realsense_align.process(frames)
                # depth = aligned_frames.get_depth_frame()

                # if not depth:
                #     raise RuntimeError("❌ Failed to get RealSense depth frame.")

                # depth_frame = np.asanyarray(depth.get_data()).astype(np.float32) / 1000.0  # in meters
            else:
                depth_frame = np.zeros_like(rgb_frame[:, :, 0], dtype=np.float32)

            return rgb_frame, depth_frame
        else:
            if self.dataset_idx >= len(self.rgb_files):
                raise IndexError("End of dataset reached.")

            rgb_path = os.path.join(self.dataset_path, self.rgb_files[self.dataset_idx])
            depth_path = os.path.join(self.dataset_path, self.depth_files[self.dataset_idx])

            rgb_frame = cv2.imread(rgb_path)
            depth_frame = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)

            self.dataset_idx += 1
            return rgb_frame, depth_frame

    def release(self):
        if self.use_webcam:
            self.cap.release()
