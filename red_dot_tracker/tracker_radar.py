import cv2
import numpy as np
from py5 import *
from servo_controller.client import ServoClient
from collections import deque
import time
import json

# === Heat-Seeking Control System Configuration ===
MAX_SERVO_ANGLE = 90  # Maximum servo angle in degrees (±90)
CONTROL_GAIN = 1.0    # Control sensitivity (0.5 = gentle, 1.5 = aggressive)
SMOOTHING_FACTOR = 0.3  # Servo movement smoothing (0.1 = very smooth, 0.9 = responsive)
RETURN_TO_CENTER_SPEED = 2.0  # Degrees per frame when returning to center
TARGET_LOST_TIMEOUT = 2.0     # Seconds before starting return-to-center
DEAD_ZONE = 20        # Pixels around center where no movement occurs

cap = None
dot_pos = None
frame_center = (0, 0)
video_w, video_h = 640, 480
radar_radius = 0
sweep_angle = 0
trail = deque(maxlen=50)
last_time = time.time()
servo = ServoClient()

# Heat-seeking control state
current_pitch_angle = 0.0  # Current servo position
current_yaw_angle = 0.0
target_pitch_angle = 0.0   # Target servo position
target_yaw_angle = 0.0
last_target_time = time.time()
system_state = "SEARCHING"  # SEARCHING, TRACKING, RETURNING

def settings():
    # Make window resizable
    size(800, 800)
    smooth()

def setup():
    global cap, frame_center
    frame_rate(60)
    cap = cv2.VideoCapture(0)
    cap.set(3, video_w)
    cap.set(4, video_h)
    frame_center = (video_w // 2, video_h // 2)
    print("=== HEAT-SEEKING CONTROL SYSTEM ACTIVE ===")
    print(f"Frame center: {frame_center}")
    print(f"Max servo angles: ±{MAX_SERVO_ANGLE}°")

def calculate_servo_angles(target_x, target_y):
    """
    Calculate pitch and yaw angles based on target position.
    Returns: (pitch_angle, yaw_angle) in degrees (±90 max)
    """
    # Calculate offset from center
    dx = target_x - frame_center[0]  # Positive = right, Negative = left
    dy = target_y - frame_center[1]  # Positive = down, Negative = up
    
    # Apply dead zone - no movement if target is close to center
    if abs(dx) < DEAD_ZONE and abs(dy) < DEAD_ZONE:
        return 0.0, 0.0
    
    # Calculate normalized distances (-1.0 to +1.0)
    # Use half frame width/height as the reference for max angle
    norm_x = dx / (video_w // 2)  # Yaw control
    norm_y = dy / (video_h // 2)  # Pitch control
    
    # Clamp to ±1.0
    norm_x = max(-1.0, min(1.0, norm_x))
    norm_y = max(-1.0, min(1.0, norm_y))
    
    # Convert to servo angles with control gain
    yaw_angle = norm_x * MAX_SERVO_ANGLE * CONTROL_GAIN
    pitch_angle = norm_y * MAX_SERVO_ANGLE * CONTROL_GAIN
    
    # Clamp to servo limits
    yaw_angle = max(-MAX_SERVO_ANGLE, min(MAX_SERVO_ANGLE, yaw_angle))
    pitch_angle = max(-MAX_SERVO_ANGLE, min(MAX_SERVO_ANGLE, pitch_angle))
    
    return pitch_angle, yaw_angle

def smooth_servo_movement():
    """
    Apply smoothing to servo movements for realistic behavior.
    """
    global current_pitch_angle, current_yaw_angle
    
    # Smoothly move current angles toward target angles
    pitch_diff = target_pitch_angle - current_pitch_angle
    yaw_diff = target_yaw_angle - current_yaw_angle
    
    current_pitch_angle += pitch_diff * SMOOTHING_FACTOR
    current_yaw_angle += yaw_diff * SMOOTHING_FACTOR

def return_to_center():
    """
    Gradually return servos to center position when no target.
    """
    global target_pitch_angle, target_yaw_angle
    
    # Move target angles toward zero
    if abs(target_pitch_angle) > 0.1:
        if target_pitch_angle > 0:
            target_pitch_angle = max(0, target_pitch_angle - RETURN_TO_CENTER_SPEED)
        else:
            target_pitch_angle = min(0, target_pitch_angle + RETURN_TO_CENTER_SPEED)
    else:
        target_pitch_angle = 0.0
    
    if abs(target_yaw_angle) > 0.1:
        if target_yaw_angle > 0:
            target_yaw_angle = max(0, target_yaw_angle - RETURN_TO_CENTER_SPEED)
        else:
            target_yaw_angle = min(0, target_yaw_angle + RETURN_TO_CENTER_SPEED)
    else:
        target_yaw_angle = 0.0

def send_servo_commands():
    """
    Send current servo positions to the servo controller.
    """
    try:
        # Only send commands if angles have changed significantly
        if abs(current_pitch_angle) > 0.5 or abs(current_yaw_angle) > 0.5:
            # pitch_command = {"axis": "pitch", "degrees": round(current_pitch_angle, 1)}
            # yaw_command = {"axis": "yaw", "degrees": round(current_yaw_angle, 1)}
            
            print(f"[SERVO] Pitch: {current_pitch_angle:.1f}°, Yaw: {current_yaw_angle:.1f}° [{system_state}]")
            
            # servo.send_command(pitch_command)
            servo.send("pitch", round(current_pitch_angle, 1))
            time.sleep(0.05)  # Small delay between commands
            # servo.send_command(yaw_command)
            servo.send("yaw", round(current_yaw_angle, 1))

    except Exception as e:
        print(f"[SERVO_ERROR] {e}")

def draw():
    global dot_pos, radar_radius, sweep_angle, last_target_time, system_state
    global target_pitch_angle, target_yaw_angle
    global width, height

    background(0)
    translate(width // 2, height // 2)
    radar_radius = min(width, height) // 2 - 50

    # === RADAR DISPLAY ===
    stroke(0, 255, 0)
    no_fill()

    # Radar rings
    for r in range(100, radar_radius+1, 100):
        ellipse(0, 0, r*2, r*2)

    # Radar spokes
    for a in range(0, 360, 45):
        x = radar_radius * cos(radians(a))
        y = radar_radius * sin(radians(a))
        line(0, 0, x, y)

    # Radar sweep line
    stroke(0, 255, 100, 120)
    stroke_weight(2)
    sweep_x = radar_radius * cos(radians(sweep_angle))
    sweep_y = radar_radius * sin(radians(sweep_angle))
    line(0, 0, sweep_x, sweep_y)
    sweep_angle = (sweep_angle + 2) % 360

    # === TARGET DETECTION ===
    ret, frame = cap.read()
    if not ret:
        # If frame read fails, show a black screen in Live Feed perhaps or skip imshow
        # For now, just return to avoid errors with frame being None
        blank_frame_for_feed = np.zeros((video_h, video_w, 3), dtype=np.uint8)
        cv2.putText(blank_frame_for_feed, "No Camera Feed", (50, video_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("Live Feed", blank_frame_for_feed)
        cv2.waitKey(1) # Essential for imshow to work
        return

    # Get processed frame with detections, and dot info
    processed_frame, dot_x, dot_y, detected_color = detect_dots(frame.copy()) # Use frame.copy() if detect_dots modifies it heavily
    dot_pos = (dot_x, dot_y) if dot_x is not None else None
    current_time = time.time()

    # Display the live feed with detection markers
    cv2.imshow("Live Feed", processed_frame)
    cv2.waitKey(1) # Essential for imshow to work, even if a frame is already shown

    if dot_pos:
        # === TARGET ACQUIRED ===
        last_target_time = current_time
        system_state = "TRACKING"
        
        # Calculate target position on radar
        dx = dot_pos[0] - frame_center[0]
        dy = dot_pos[1] - frame_center[1]
        angle = atan2(dy, dx)
        dist_factor = dist(0, 0, dx, dy) / (video_w // 2)
        r = min(radar_radius, dist_factor * radar_radius)

        x = r * cos(angle)
        y = r * sin(angle)
        trail.append((x, y))

        # === HEAT-SEEKING CONTROL ===
        target_pitch_angle, target_yaw_angle = calculate_servo_angles(dot_pos[0], dot_pos[1])
        
        # Apply servo smoothing and send commands
        smooth_servo_movement()
        send_servo_commands()

        # === VISUAL FEEDBACK ===
        # Target dot - color based on detected_color
        if detected_color == "RED":
            fill(255, 0, 0)
        elif detected_color == "YELLOW":
            fill(255, 255, 0)
        else:
            fill(128, 128, 128) # Default if color unknown (should not happen here)
        no_stroke()
        ellipse(x, y, 20, 20)
        
        # Targeting crosshairs (can also be color-coded if desired)
        stroke(255, 0, 0)
        stroke_weight(2)
        line(x-15, y, x+15, y)
        line(x, y-15, x, y+15)
        
        # Target lock indicator (can also be color-coded)
        stroke(255, 255, 0)
        no_fill()
        ellipse(x, y, 40, 40)

        # Status display
        fill(0, 255, 0)
        text_size(16)
        text_align(LEFT, TOP)
        text(f"🎯 TARGET ({detected_color}) LOCKED", -radar_radius + 10, -radar_radius + 10)
        text(f"Pitch: {current_pitch_angle:.1f}°", -radar_radius + 10, -radar_radius + 30)
        text(f"Yaw: {current_yaw_angle:.1f}°", -radar_radius + 10, -radar_radius + 50)
        text(f"Distance: {dist_factor:.2f}", -radar_radius + 10, -radar_radius + 70)

    else:
        # === NO TARGET ===
        if current_time - last_target_time > TARGET_LOST_TIMEOUT:
            system_state = "RETURNING"
            return_to_center()
            smooth_servo_movement()
            send_servo_commands()
        else:
            system_state = "SEARCHING"
        
        # Status display
        fill(255, 100, 0)
        text_size(16)
        text_align(LEFT, TOP)
        if system_state == "SEARCHING":
            text("🔍 SEARCHING FOR TARGET...", -radar_radius + 10, -radar_radius + 10)
        else:
            text("🔄 RETURNING TO CENTER...", -radar_radius + 10, -radar_radius + 10)
        text(f"Pitch: {current_pitch_angle:.1f}°", -radar_radius + 10, -radar_radius + 30)
        text(f"Yaw: {current_yaw_angle:.1f}°", -radar_radius + 10, -radar_radius + 50)

    # === TARGETING TRAIL ===
    if current_time - last_target_time > 3.0:
        trail.clear()
    else:
        no_fill()
        stroke(255, 100, 0, 180)
        stroke_weight(2)
        begin_shape()
        for tx, ty in trail:
            vertex(tx, ty)
        end_shape()

    # === CENTER CROSSHAIRS ===
    stroke(0, 255, 0, 100)
    stroke_weight(1)
    line(-20, 0, 20, 0)
    line(0, -20, 0, 20)

def detect_dots(input_frame): # Renamed frame to input_frame to avoid confusion
    hsv = cv2.cvtColor(input_frame, cv2.COLOR_BGR2HSV)
    output_frame = input_frame # We will draw on this frame
    
    # --- Color Ranges (HSV) ---
    # Red (wraps around, so two ranges)
    lower_red1 = np.array([0, 150, 100])     # Adjusted saturation and value
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([165, 150, 100])   # Adjusted hue, saturation, and value
    upper_red2 = np.array([180, 255, 255])

    # Yellow
    lower_yellow = np.array([20, 150, 100])  # Adjusted saturation and value
    upper_yellow = np.array([35, 255, 255]) # Widened hue range slightly

    # --- Mask Creation ---
    mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_r1, mask_r2)

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # --- Morphological Operations (separate for each color before combining) ---
    kernel = np.ones((5,5),np.uint8) # Define kernel once
    
    # Red dot processing
    processed_mask_red = cv2.erode(mask_red, kernel, iterations=1) # Less aggressive erode
    processed_mask_red = cv2.dilate(processed_mask_red, kernel, iterations=2)
    
    # Yellow dot processing
    processed_mask_yellow = cv2.erode(mask_yellow, kernel, iterations=1)
    processed_mask_yellow = cv2.dilate(processed_mask_yellow, kernel, iterations=2)

    # --- Contour Detection and Filtering (function for reusability) ---
    def find_best_dot(color_mask, color_name, frame_to_draw_on): # Added frame_to_draw_on
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_dot_info = None # Store full info for drawing
        max_area = 0

        if not contours:
            return None

        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < 150: 
                continue

            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            
            if radius < 8: 
                continue

            if radius > 0:
                circularity = area / (np.pi * (radius ** 2))
                if circularity < 0.65: 
                    continue
            else:
                continue 
            
            x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(contour)
            aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
            if not (0.7 < aspect_ratio < 1.4): 
                continue

            if area > max_area: 
                max_area = area
                # Store all details needed for drawing and logic
                best_dot_info = (int(x), int(y), int(radius), area, circularity, aspect_ratio, color_name)
        
        if best_dot_info:
            # Draw on the frame if a dot is found
            bx, by, br, _, _, _, bcolor = best_dot_info
            draw_color = (0, 255, 0) # Green for generic detection
            if bcolor == "RED":
                draw_color = (0, 0, 255) # Blue for Red dot (BGR format for OpenCV)
            elif bcolor == "YELLOW":
                draw_color = (0, 255, 255) # Cyan for Yellow dot (BGR)
            
            cv2.circle(frame_to_draw_on, (bx, by), br, draw_color, 2)
            cv2.circle(frame_to_draw_on, (bx,by), 2, (0,0,255), 2) # center point
            cv2.putText(frame_to_draw_on, f"{bcolor} R:{br}", (bx - br, by - br - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw_color, 1)
            # Add bounding box for aspect ratio debug
            # cv2.rectangle(frame_to_draw_on,(x_rect,y_rect),(x_rect+w_rect,y_rect+h_rect), (255,0,255) ,1)
            # print(f\"[DEBUG] Best {color_name} dot: Pos=({bx},{by}), R={br}, Area={best_dot_info[3]:.0f}, Circ={best_dot_info[4]:.2f}, AR={best_dot_info[5]:.2f}\")
            return best_dot_info
        return None

    # --- Prioritized Dot Detection ---
    # 1. Look for Red dot
    # Pass output_frame to find_best_dot so it can draw on it
    red_dot_info = find_best_dot(processed_mask_red, "RED", output_frame) 
    if red_dot_info:
        print(f"[DETECTION] Tracking RED dot at ({red_dot_info[0]}, {red_dot_info[1]}), R:{red_dot_info[2]}")
        # Return modified frame, x, y, color_name
        return output_frame, red_dot_info[0], red_dot_info[1], "RED" 

    # 2. If no Red dot, look for Yellow dot
    yellow_dot_info = find_best_dot(processed_mask_yellow, "YELLOW", output_frame)
    if yellow_dot_info:
        print(f"[DETECTION] Tracking YELLOW dot at ({yellow_dot_info[0]}, {yellow_dot_info[1]}), R:{yellow_dot_info[2]}")
        return output_frame, yellow_dot_info[0], yellow_dot_info[1], "YELLOW"

    return output_frame, None, None, None # Return frame even if no dot found

def exit_handler():
    print("Exiting script...")
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    if servo is not None:
        servo.close()
    print("Cleanup complete. Bye!")

if __name__ == '__main__':
    def py5_exit_hook():
        exit_handler()

    def exit():
        exit_handler()

    run_sketch()
