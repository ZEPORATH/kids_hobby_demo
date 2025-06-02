# servo_controller/server.py

import socket
# import threading # Removed threading
import RPi.GPIO as GPIO
import time
import json
import driver # Import the driver module

# === Configuration ===
PITCH_PIN = 17
YAW_PIN = 18
COMMAND_TIMEOUT_MS = 30 # Milliseconds - discard commands older than this

# Calibrated value from driver.py
# SECONDS_FOR_360_DEG_AT_FULL_SPEED = 0.7165 # Already in driver
# FULL_MOVE_SPEED = 1.0 # Already in driver

GPIO.setmode(GPIO.BCM)
GPIO.setup(PITCH_PIN, GPIO.OUT)
GPIO.setup(YAW_PIN, GPIO.OUT)

# Servo objects are now accessed via the driver module (e.g., driver.pitch)
# Constants like SECONDS_FOR_360_DEG_AT_FULL_SPEED are also in driver

# === Server Logic ===
# Removed handle_client function as it's not needed for UDP in this simple case

def main():
    host, port = "0.0.0.0", 9999
    # Changed to SOCK_DGRAM for UDP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    print(f"[SERVER] UDP Listening on {host}:{port}")
    print(f"[INFO] Using {driver.SECONDS_FOR_360_DEG_AT_FULL_SPEED}s for 360 deg rotation (from driver module).")
    print(f"[INFO] Command timeout set to {COMMAND_TIMEOUT_MS}ms.")

    try:
        # Initialize servos - this happens when driver.py is imported.
        # driver.pitch.start(0) # Already in driver.py's global scope
        # driver.yaw.start(0)   # Already in driver.py's global scope

        while True:
            try:
                # For UDP, we receive data and the client's address
                data, addr = server_socket.recvfrom(1024) # Use recvfrom for UDP
                # No need to print client connected for UDP as it's connectionless
                # print(f"[CLIENT CONNECTED] {addr}") 

                current_server_time = time.time()
                
                msg_str = data.decode()
                # Potentially reduce print frequency for performance if needed
                # print(f"[RECV] From {addr}: {msg_str}") 
                msg = json.loads(msg_str)
                
                # --- Timestamp Check ---
                cmd_timestamp = msg.get("timestamp")
                if cmd_timestamp is None:
                    print(f"[WARN] Received command from {addr} without timestamp. Discarding.")
                    continue
                
                age_ms = (current_server_time - cmd_timestamp) * 1000
                if age_ms < 0: # Clock sync issue or future timestamp
                    print(f"[WARN] Command from {addr} has future or invalid timestamp (age: {age_ms:.1f}ms). Processing anyway for now, but check clocks.")
                elif age_ms > COMMAND_TIMEOUT_MS:
                    print(f"[STALE_CMD] Discarding command from {addr} (age: {age_ms:.1f}ms > {COMMAND_TIMEOUT_MS}ms)")
                    continue
                # else:
                    # print(f"[VALID_CMD] Processing command from {addr} (age: {age_ms:.1f}ms)")

                axis = msg.get("axis")
                degrees = float(msg.get("degrees", 0))

                target_servo_obj = None
                if axis == "pitch":
                    target_servo_obj = driver.pitch
                    # print(f"[COMMAND] Pitch servo move by {degrees} degrees.")
                elif axis == "yaw":
                    target_servo_obj = driver.yaw
                    # print(f"[COMMAND] Yaw servo move by {degrees} degrees.")
                elif axis == "stop":
                    # print(f"[COMMAND] Stop all servos.")
                    driver.stop_all_servos()
                    continue 
                else:
                    print(f"[WARN] Unknown axis: {axis} from {addr}")
                    continue
                
                if target_servo_obj:
                    driver.move_by_angle(target_servo_obj, degrees)

            except json.JSONDecodeError:
                # For UDP, we might not have msg_str if decode fails early, or it might be partial
                print(f"[ERROR] Invalid JSON received from {addr}.") # Simplified error for UDP
            # Removed ConnectionResetError as it's TCP specific
            except Exception as e:
                print(f"[ERROR] Processing UDP packet from {addr}: {e}")
                # For UDP, server continues to run after an error with a packet

    except KeyboardInterrupt:
        print("[SERVER] Shutting down...")
    finally:
        print("[CLEANUP] Stopping all servos before exit...")
        if hasattr(driver, 'stop_all_servos'):
            driver.stop_all_servos()
        time.sleep(0.1) # Give servos time to stop
        if hasattr(driver, 'pitch') and driver.pitch is not None:
            driver.pitch.stop()
        if hasattr(driver, 'yaw') and driver.yaw is not None:
            driver.yaw.stop()
        GPIO.cleanup()
        server_socket.close() # Close the socket
        print("[CLEANUP] GPIO cleanup complete. Server exited.")

if __name__ == '__main__':
    main()
