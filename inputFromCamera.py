### File: inputFromCamera.py
import cv2
import numpy as np
import os
import time

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
                # Placeholder: integrate actual RGBD camera API here
                depth_frame = np.random.uniform(0.3, 1.5, rgb_frame.shape[:2]).astype(np.float32)
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
