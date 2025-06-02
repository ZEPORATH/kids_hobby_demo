# Radar Visualizer (180° Sweep) with Pygame and Serial Input

import pygame
import serial
import math
import argparse
import time # For timestamp in console log
import os # For os.path.dirname(__file__)
from collections import deque # For efficient fixed-size buffer
import threading
import queue # For thread-safe queue

# --- Configuration ---
DEFAULT_SERIAL_PORT = "/dev/ttyACM1"  # Default for Linux, change if needed (e.g., "COM3" for Windows)
BAUD_RATE = 9600

# Initial Screen Dimensions (will be updated if maximized or resized)
INITIAL_SCREEN_WIDTH = 800
INITIAL_SCREEN_HEIGHT = 600

# Global variables for screen dimensions and related calculations
SCREEN_WIDTH = INITIAL_SCREEN_WIDTH
SCREEN_HEIGHT = INITIAL_SCREEN_HEIGHT
CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT - 100 # Place center near bottom for upward sweep display
MAX_DISPLAY_RADIUS = SCREEN_HEIGHT - 120 # Max radius for drawing on screen

# MAX_SENSOR_DISTANCE_CM is effectively replaced by PLOT_MAX_DISTANCE_CM for display scaling.
# We still might need MAX_SENSOR_DISTANCE_CM if sensor has a different practical limit we need to clamp against before filtering.
# For now, let's assume PLOT_MAX_DISTANCE_CM is the primary ceiling for valid, displayable data.
PLOT_MAX_DISTANCE_CM = 200.0    # User-defined limit: Do not plot points beyond this distance.
                                # This will also be used for scaling display radius.
# ARDUINO_MAX_STEPS: Max step value Arduino sends for one sweep direction (0-180 degrees phys mapping)
# Based on user feedback, this is 50 for their 0-50 step range.
ARDUINO_MAX_STEPS = 70.0      # Overriding .ino read for now based on user input.

# Colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)
# YELLOW = (255, 255, 0) # No longer used for mean point

# --- Point Display Configuration ---
POINT_DISPLAY_TTL_SEC = 2.0 # How long individual points stay visible
INDIVIDUAL_POINT_RADIUS = 2 # Radius for drawing individual points
POINT_BUFFER_MAX_SIZE = 50 # Max number of points to keep in the display buffer

# --- Global Variables for Data Exchange & Thread Control ---
data_queue = queue.Queue() # Thread-safe queue for points from serial thread to main thread
stop_serial_thread = threading.Event() # Event to signal the serial thread to stop

# --- Global Variables for Pygame Display (updated by main thread from queue) ---
recent_points_buffer = deque(maxlen=POINT_BUFFER_MAX_SIZE) # Capped buffer size
current_sweep_angle_rad = math.radians(180) # Start sweep from the left
font = None

def update_display_parameters(width, height):
    global SCREEN_WIDTH, SCREEN_HEIGHT, CENTER_X, CENTER_Y, MAX_DISPLAY_RADIUS
    SCREEN_WIDTH = width
    SCREEN_HEIGHT = height
    CENTER_X = SCREEN_WIDTH // 2
    CENTER_Y = SCREEN_HEIGHT - max(100, int(SCREEN_HEIGHT * 0.15)) # Keep radar base some distance from bottom
    MAX_DISPLAY_RADIUS = min(CENTER_Y - 20, SCREEN_WIDTH // 2 - 20) # Ensure radar fits
    print(f"Window resized to: {SCREEN_WIDTH}x{SCREEN_HEIGHT}. New MAX_DISPLAY_RADIUS: {MAX_DISPLAY_RADIUS}")

def serial_reader_thread_func(port_name_arg, baud_rate_arg, stop_event, data_q):
    """Thread function to read from serial, parse data, and put it on a queue."""
    s_port = None
    try:
        s_port = serial.Serial(port_name_arg, baud_rate_arg, timeout=0.1)
        print(f"Serial thread: Successfully connected to {port_name_arg}")
    except serial.SerialException as e:
        print(f"Serial thread: Error opening serial port {port_name_arg}: {e}")
        return # Exit thread if port cannot be opened

    while not stop_event.is_set():
        if not s_port.is_open:
            # print("Serial thread: Port closed, attempting to reopen or exiting...") # Or just exit
            stop_event.set() # Ensure loop terminates if port is externally closed
            break

        try:
            if s_port.in_waiting > 0:
                line = s_port.readline().decode('utf-8').strip()
                # print(f"Serial thread RX: {line}") # Print raw incoming line
                
                if not line or ',' not in line: 
                    if line and not line.startswith("Arduino Radar") and not line.startswith("Radar Ready"):
                        # Potentially print here if you want to see these specific skipped lines too,
                        # but the raw RX print above already covers it.
                        pass 
                    continue # Next iteration of the while loop
                
                parts = line.split(',')
                if len(parts) != 2: 
                    # print(f"Serial thread: Skipping malformed data (expected 2 parts): {line}") # Already printed raw
                    continue

                try:
                    step = float(parts[0])
                    raw_distance_cm = float(parts[1])
                except ValueError:
                    # if not any(header in line for header in ["Arduino Radar", "Radar Ready", "Angle", "Distance", "Format"]):
                         # print(f"Serial thread: Skipping header/non-numeric data: {line}") # Already printed raw
                    continue
                
                # Filter points beyond PLOT_MAX_DISTANCE_CM or if distance is 0 (invalid)
                if raw_distance_cm <= 0 or raw_distance_cm > PLOT_MAX_DISTANCE_CM:
                    continue 

                # Angle calculation: step (0-50) maps to angle_deg (0-180)
                angle_deg = (step / ARDUINO_MAX_STEPS) * 180.0
                angle_deg = max(0, min(angle_deg, 180.0)) # Ensure it's within 0-180 bounds
                point_angle_rad = math.radians(angle_deg)
                        
                # Distance scaling for display:
                # raw_distance_cm (0 to PLOT_MAX_DISTANCE_CM) maps to radius (0 to MAX_DISPLAY_RADIUS)
                # No need for further clamping of raw_distance_cm here as it's already filtered.
                point_radius_on_screen = (raw_distance_cm / PLOT_MAX_DISTANCE_CM) * MAX_DISPLAY_RADIUS
                point_radius_on_screen = max(0, min(point_radius_on_screen, MAX_DISPLAY_RADIUS)) # Ensure screen radius is valid
                
                new_point_dict = {
                    'angle_rad': point_angle_rad,
                    'radius_screen': point_radius_on_screen,
                    'dist_cm': raw_distance_cm, # Store the filtered, valid raw distance
                    'time': time.time() 
                }
                # Put data dictionary and current angle for sweep onto the queue
                data_q.put({'point_data': new_point_dict, 'sweep_angle': point_angle_rad})
            else:
                time.sleep(0.005) # Sleep briefly if no data to avoid busy-waiting
        
        except serial.SerialException as se:
            print(f"Serial thread: SerialException: {se}. Stopping thread.")
            stop_event.set()
            break # Exit while loop
        except UnicodeDecodeError:
            # This can happen with noise or baud rate issues. Less critical to spam console for each.
            pass # print(f"Serial thread: Error decoding serial data.")
        except Exception as e:
            print(f"Serial thread: Unexpected error: {e} (Line: '{line if 'line' in locals() else 'N/A'}')")
            # Depending on error, might want to stop_event.set()
            time.sleep(0.01) # Avoid tight loop on continuous error
    
    if s_port and s_port.is_open:
        print("Serial thread: Closing serial port.")
        s_port.close()
    print("Serial reader thread finished.")

def process_data_from_queue():
    """Process data from the queue filled by the serial thread."""
    global current_sweep_angle_rad, recent_points_buffer
    while not data_queue.empty():
        try:
            item = data_queue.get_nowait()
            point = item['point_data']
            current_sweep_angle_rad = item['sweep_angle'] # Update sweep angle from thread data
            recent_points_buffer.append(point)
            data_queue.task_done() # Indicate item processing is complete
        except queue.Empty:
            break # Should not happen if we check data_queue.empty() first, but good practice
        except Exception as e:
            print(f"Error processing data from queue: {e}")

def draw_radar_grid(screen):
    if not font or MAX_DISPLAY_RADIUS <= 0: return
    screen.fill(BLACK)
    # Grid distance markers scaled to PLOT_MAX_DISTANCE_CM
    for r_factor in [0.25, 0.5, 0.75, 1.0]:
        # Displayed distance for the marker
        marker_dist_cm = int(r_factor * PLOT_MAX_DISTANCE_CM)
        # Radius on screen for this marker arc
        radius_on_screen = int(r_factor * MAX_DISPLAY_RADIUS)
        if radius_on_screen <=0: continue
        pygame.draw.arc(screen, DARK_GREEN, [CENTER_X - radius_on_screen, CENTER_Y - radius_on_screen, radius_on_screen * 2, radius_on_screen * 2], 0, math.pi, 1)
        text_surface = font.render(f"{marker_dist_cm}cm", True, GREEN)
        screen.blit(text_surface, (CENTER_X + radius_on_screen + 5, CENTER_Y - text_surface.get_height()//2))

    for angle_deg_marker in range(0, 181, 30):
        angle_rad_marker = math.radians(angle_deg_marker)
        end_x = CENTER_X + math.cos(angle_rad_marker) * MAX_DISPLAY_RADIUS
        end_y = CENTER_Y - math.sin(angle_rad_marker) * MAX_DISPLAY_RADIUS
        pygame.draw.line(screen, DARK_GREEN, (CENTER_X, CENTER_Y), (end_x, end_y), 1)
        text_surface = font.render(f"{angle_deg_marker}°", True, GREEN)
        text_x = CENTER_X + math.cos(angle_rad_marker) * (MAX_DISPLAY_RADIUS + 15)
        text_y = CENTER_Y - math.sin(angle_rad_marker) * (MAX_DISPLAY_RADIUS + 15)
        if angle_deg_marker == 90: text_x -= text_surface.get_width() // 2; text_y -= text_surface.get_height()
        elif angle_deg_marker > 90: text_x -= text_surface.get_width(); text_y -= text_surface.get_height() // 2
        else: text_y -= text_surface.get_height() // 2
        screen.blit(text_surface, (text_x, text_y))

def draw_detections(screen):
    global recent_points_buffer # We will modify this deque
    if MAX_DISPLAY_RADIUS <= 0: return

    # Draw sweep line
    sweep_x = CENTER_X + math.cos(current_sweep_angle_rad) * MAX_DISPLAY_RADIUS
    sweep_y = CENTER_Y - math.sin(current_sweep_angle_rad) * MAX_DISPLAY_RADIUS
    pygame.draw.line(screen, GREEN, (CENTER_X, CENTER_Y), (sweep_x, sweep_y), 2)

    current_time = time.time()
    points_to_keep_in_buffer = deque() # Create a new deque for points that are not too old

    for point in recent_points_buffer:
        point_age = current_time - point['time']
        if point_age < POINT_DISPLAY_TTL_SEC:
            # Draw this point
            px = CENTER_X + math.cos(point['angle_rad']) * point['radius_screen']
            py = CENTER_Y - math.sin(point['angle_rad']) * point['radius_screen']
            pygame.draw.circle(screen, RED, (int(px), int(py)), INDIVIDUAL_POINT_RADIUS)
            points_to_keep_in_buffer.append(point) # Add to the new buffer if it was drawn
    
    recent_points_buffer = points_to_keep_in_buffer # Replace old buffer with culled one

def main(port_name, start_maximized):
    global font # ARDUINO_MAX_STEPS is now hardcoded
    pygame.init()
    
    # ARDUINO_MAX_STEPS is now hardcoded, so .ino reading logic is removed for it.
    # print(f"Main: Using hardcoded ARDUINO_MAX_STEPS = {ARDUINO_MAX_STEPS}")

    pygame.display.set_caption("Radar Live Points Display (Threaded)")
    font = pygame.font.Font(None, 22)

    display_flags = pygame.RESIZABLE
    if start_maximized:
        try:
            info = pygame.display.Info()
            SCREEN_WIDTH, SCREEN_HEIGHT = info.current_w, info.current_h
        except: SCREEN_WIDTH, SCREEN_HEIGHT = INITIAL_SCREEN_WIDTH, INITIAL_SCREEN_HEIGHT
    else: SCREEN_WIDTH, SCREEN_HEIGHT = INITIAL_SCREEN_WIDTH, INITIAL_SCREEN_HEIGHT

    update_display_parameters(SCREEN_WIDTH, SCREEN_HEIGHT)
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), display_flags)

    # Start the serial reader thread
    serial_thread = threading.Thread(
        target=serial_reader_thread_func, 
        args=(port_name, BAUD_RATE, stop_serial_thread, data_queue),
        daemon=True # Thread will exit when main program exits
    )
    serial_thread.start()
    print(f"Main: Serial reader thread initiated for port {port_name}.")

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                update_display_parameters(event.w, event.h)
                screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), display_flags)

        process_data_from_queue() # Get data from serial thread
        
        draw_radar_grid(screen)
        draw_detections(screen)

        pygame.display.flip()
        clock.tick(30) 

    print("Main: Exiting Pygame loop. Signaling serial thread to stop...")
    stop_serial_thread.set()
    # serial_thread.join(timeout=2) # Wait for thread to finish, optional if daemon=True
    # print("Main: Serial thread joined (or timed out).")
    
    # No need to explicitly close serial_port here, thread handles its instance.
    pygame.quit()
    print("Main: Pygame quit successfully.")

if __name__ == '__main__':
    # Required for os.path.dirname(__file__)
    import os 

    parser = argparse.ArgumentParser(description="Radar Live Points Display (Threaded)")
    parser.add_argument('--port', type=str, default=DEFAULT_SERIAL_PORT,
                        help=f"Serial port for Arduino (default: {DEFAULT_SERIAL_PORT})")
    parser.add_argument('--start_maximized', action='store_true',
                        help="Start the window maximized to the screen size.")
    args = parser.parse_args()
    main(args.port, args.start_maximized)
