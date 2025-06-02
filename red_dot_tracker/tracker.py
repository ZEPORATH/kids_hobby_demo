import cv2
import numpy as np

def get_direction(cx, cy, frame_center):
    x, y = frame_center
    direction = []

    if cx < x - 50:
        direction.append("LEFT")
    elif cx > x + 50:
        direction.append("RIGHT")

    if cy < y - 50:
        direction.append("UP")
    elif cy > y + 50:
        direction.append("DOWN")

    if not direction:
        return "CENTERED"
    return " & ".join(direction)

cap = cv2.VideoCapture(0)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
frame_center = (frame_width // 2, frame_height // 2)

print("Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Define red color range (two parts due to HSV wraparound)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)

        if radius > 10:
            M = cv2.moments(largest)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)
            direction = get_direction(cx, cy, frame_center)
            cv2.putText(frame, f"Direction: {direction}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            print(f"Tracking dot at ({cx}, {cy}) → Command: {direction}")

    cv2.imshow("Red Dot Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
