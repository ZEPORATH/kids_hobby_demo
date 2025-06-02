import RPi.GPIO as GPIO
import time

# === Configuration ===
PITCH_PIN = 17  # GPIO pin for pitch flap servo
YAW_PIN = 18    # GPIO pin for yaw flap servo

# !!! --- CALIBRATION REQUIRED --- !!!
# Previous value: 0.6667. User reports that commanding 360 deg (which ran motor for 0.6667s)
# resulted in a turn ~25 deg short (i.e., ~335 deg actual).
# New calculation: 0.6667s * (360_desired_deg / 335_actual_deg) = 0.6667 * 1.0746... = 0.71645...
# Rounded to 0.7165
SECONDS_FOR_360_DEG_AT_FULL_SPEED = 0.7165

# Speed used for "angle" movements. 1.0 is full speed.
FULL_MOVE_SPEED = 1.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(PITCH_PIN, GPIO.OUT)
GPIO.setup(YAW_PIN, GPIO.OUT)

pitch = GPIO.PWM(PITCH_PIN, 50)  # 50Hz
yaw = GPIO.PWM(YAW_PIN, 50)

# Start servos at "stop" duty cycle (approx 7.5% for continuous rotation servos)
# This is the command to make them stationary.
pitch.start(0)
yaw.start(0)

# === Functions ===

def set_speed(pwm, speed):
    """
    Sets the speed of a continuous rotation servo.
    speed: float in range [-1.0, 1.0]
        -1.0 = full speed one direction (e.g., CW)
         0.0 = stop (sends 0% duty cycle as per user request)
         1.0 = full speed other direction (e.g., CCW)
    """
    speed = max(min(speed, 1.0), -1.0)

    if speed == 0.0:
        duty = 0.0
    else:
        # Duty cycle for continuous servos: ~7.5% is neutral/stop,
        # 5% is full one way, 10% is full other way.
        # This formula maps non-zero speeds to the 5-10% range (excluding 7.5% itself).
        duty = 7.5 + (speed * 2.5)

    pwm.ChangeDutyCycle(duty)
    # print(f"[SPEED] Servo set to speed: {speed:.2f} → duty: {duty:.2f}%") # Optional: uncomment for debugging

def stop_all_servos():
    # print("[STOP] Commanding all servos to stop (speed 0.0).") # Optional: uncomment for debugging
    set_speed(pitch, 0)
    set_speed(yaw, 0)

def move_by_angle(pwm, degrees):
    """
    Moves the servo by a specified number of degrees.
    This is an approximation for continuous rotation servos based on timed movement.
    Requires SECONDS_FOR_360_DEG_AT_FULL_SPEED to be calibrated.

    degrees: The number of degrees to turn. Positive for one direction, negative for the other.
    """
    if SECONDS_FOR_360_DEG_AT_FULL_SPEED <= 0:
        print("[ERR] SECONDS_FOR_360_DEG_AT_FULL_SPEED is not calibrated correctly. Aborting move.")
        return

    if degrees == 0:
        set_speed(pwm, 0) # Ensure it's stopped if 0 degrees is requested
        # print(f"[ANGLE] Requested 0 degrees movement. Servo commanded to stop.") # Optional
        return

    direction_speed = FULL_MOVE_SPEED if degrees > 0 else -FULL_MOVE_SPEED
    duration = (abs(degrees) / 360.0) * SECONDS_FOR_360_DEG_AT_FULL_SPEED / abs(direction_speed)

    # print(f"[ANGLE] Moving by {degrees:.1f} deg: Speed={direction_speed:.2f}, Duration={duration:.3f}s") # Optional
    
    set_speed(pwm, direction_speed)
    time.sleep(duration)
    set_speed(pwm, 0) # Command servo to stop after the timed movement
    # print(f"[ANGLE] Movement by {degrees:.1f} degrees complete. Servo commanded to stop.") # Optional


def main():
    print("=== Servo Driver for Continuous Rotation Servos (Angle Control) ===")
    print(f"CALIBRATION: Using {SECONDS_FOR_360_DEG_AT_FULL_SPEED}s for 360 deg rotation.")
    print("!!! If servos don't stop at speed 0, fine-tune the 7.5 in set_speed() !!!")
    print("Controls:")
    print("  p <degrees>  (e.g., 'p 90' or 'p -45')")
    print("  y <degrees>  (e.g., 'y 180' or 'y -30')")
    print("  stop         (stop both servos immediately)")
    print("  exit or Ctrl+C (quit)")

    try:
        while True:
            raw_input = input(">> ").strip().lower()
            if raw_input in ("exit", "quit"):
                break
            elif raw_input == "stop":
                stop_all_servos()
                continue

            parts = raw_input.split()
            if len(parts) != 2:
                print("[ERR] Invalid format. Use: <axis> <degrees> (e.g., 'p 90') or 'stop'")
                continue

            axis, value_str = parts
            try:
                degrees = float(value_str)
            except ValueError:
                print("[ERR] Invalid number for degrees. Please enter a numeric value.")
                continue

            target_servo = None
            if axis == "p":
                target_servo = pitch
            elif axis == "y":
                target_servo = yaw
            else:
                print("[ERR] Unknown axis. Use 'p' for pitch or 'y' for yaw.")
                continue
            
            if target_servo:
                move_by_angle(target_servo, degrees)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C received, stopping servos...")
    finally:
        print("[CLEANUP] Ensuring servos are commanded to stop before final PWM release.")
        stop_all_servos() # Ensure servos are commanded to stop (speed 0)
        time.sleep(0.1)   # Short pause to allow stop command to be processed
        pitch.stop()      # Release PWM signal generation
        yaw.stop()        # Release PWM signal generation
#        GPIO.cleanup()
#        print("[CLEANUP] GPIO cleanup complete. Exiting.")

# if __name__ == "__main__":
#     main()
