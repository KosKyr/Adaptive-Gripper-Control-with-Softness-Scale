### File: detectGripper.py
import cv2
import numpy as np

def find_centroids_of_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Relaxed red color ranges
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Show red mask for debugging
    cv2.imshow("Red Mask", red_mask)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centers = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 200:
            continue
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append((cx, cy))

    if centers:
        print(f"✅ Detected {len(centers)} red point(s).")
    else:
        print("❌ No red points detected.")

    return red_mask, centers

def compute_distance_between_centers(centers):
    if len(centers) >= 2:
        centers = sorted(centers, key=lambda c: c[0])
        c1, c2 = centers[0], centers[1]
        distance = abs(c2[0] - c1[0])
        return distance, c1, c2
    return None, None, None

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        original = frame.copy()
        red_mask, centers = find_centroids_of_red(frame)
        distance, c1, c2 = compute_distance_between_centers(centers)

        # Create an overlay where only red spots are kept, rest black
        red_overlay = cv2.bitwise_and(original, original, mask=red_mask)

        if distance is not None:
            cv2.line(frame, c1, c2, (255, 0, 0), 2)
            mid_x = (c1[0] + c2[0]) // 2
            mid_y = (c1[1] + c2[1]) // 2
            cv2.putText(frame, f"Width: {distance}px", (mid_x - 50, mid_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(red_overlay, c1, 5, (0, 255, 0), -1)
            cv2.circle(red_overlay, c2, 5, (0, 255, 0), -1)

        cv2.imshow("Red Detection", frame)
        cv2.imshow("Filtered Red Only", red_overlay)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()