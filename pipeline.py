### File: pipeline.py
import cv2
import numpy as np
from configYOLO import classNames, FONT, FONT_SCALE, THICKNESS, load_yolo_model
from inputFromCamera import InputFromCamera
from dimension_sender import DimensionSender
from configDepth import calculate_average_depth, normalize_depth_frame

class Pipeline():
    def __init__(self, label_color, box_color, mask_color, alpha, pad, confidence_threeshold, yolo_weights_path, dataset_path, use_data_set, use_rgbd=False, filter_classes=None):
        self.label_color = label_color[::-1]
        self.box_color = (0, 255, 0)
        self.mask_color = mask_color[::-1]
        self.alpha = alpha
        self.pad = pad
        self.confidence_threeshold = confidence_threeshold
        self.yolo_weights_path = yolo_weights_path
        self.dataset_path = dataset_path
        self.use_data_set = use_data_set
        self.use_rgbd = use_rgbd
        self.filter_classes = filter_classes if filter_classes else []
        print("Pipeline Initialized")

    def detect_red_centers(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Stricter red thresholds
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        centers = []
        for cnt in contours[:2]:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        if len(centers) == 2:
            cv2.line(frame, centers[0], centers[1], (255, 0, 0), 2)
        print(f"Red centers detected: {len(centers)}")
        return frame, centers

    def main(self):
        input_source = InputFromCamera(use_webcam=not self.use_data_set, dataset_path=self.dataset_path, use_rgbd=self.use_rgbd)
        model = load_yolo_model(self.yolo_weights_path)
        dimension_sender = DimensionSender(port='COM3')

        while True:
            try:
                rgb_frame, depth_frame = input_source.get_frame()

                rgb_frame, red_centers = self.detect_red_centers(rgb_frame)

                results = model(rgb_frame, stream=True)

                if len(red_centers) == 2:
                    gripper_width = np.linalg.norm(np.array(red_centers[0]) - np.array(red_centers[1]))

                for r in results:
                    #print(f"Detected {len(r.boxes)} boxes")
                    boxes = r.boxes

                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])

                        if confidence < self.confidence_threeshold:
                            continue

                        cls = int(box.cls[0])
                        class_name = classNames[cls]

                        if self.filter_classes and class_name not in self.filter_classes:
                            continue

                        #print(f"Drawing box for {class_name} at {x1},{y1},{x2},{y2}")
                        cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), self.box_color, 2)

                        width_px = x2 - x1
                        height_px = y2 - y1

                        depth_crop = depth_frame[y1:y2, x1:x2]
                        depth_m = calculate_average_depth(depth_crop)

                        fx = 600
                        fy = 600

                        length_cm = (width_px * depth_m / fx) * 100
                        width_cm = (height_px * depth_m / fy) * 100

                        print(f"[{class_name}] Length: {length_cm:.2f} cm, Width: {width_cm:.2f} cm")

                        label_x, label_y = x1, y1 - 10
                        info_text = f"{class_name}: {length_cm:.1f} x {width_cm:.1f} cm"
                        cv2.putText(rgb_frame, info_text, (label_x, label_y), FONT, FONT_SCALE, self.label_color, THICKNESS)

                        if len(red_centers) == 2:
                            line_start, line_end = red_centers
                            # Check if center of bounding box is on or near the line
                            obj_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                            # Approximate intercept check by horizontal span
                            if x1 < line_start[0] < x2 or x1 < line_end[0] < x2:
                                if width_px <= gripper_width:
                                    print("✅ Object can be gripped.")
                                else:
                                    print("❌ Object too large to grip.")

                cv2.imshow("Object Detection with Red Centers", rgb_frame)
                if cv2.waitKey(1) == ord('q'):
                    break

            except Exception as e:
                print(f"Error: {e}")
                break

        input_source.release()
        cv2.destroyAllWindows()
        dimension_sender.close()
