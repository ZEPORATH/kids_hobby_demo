# Kids Hobby Demo

This project showcases a couple of fun robotics demonstrations for kids, designed to be engaging and educational.

## Projects

### 1. Servo Ultrasonic Scanner

**Description:**
This project uses an Arduino microcontroller connected to an ultrasonic distance sensor and a servo motor. The servo sweeps the ultrasonic sensor, scanning the surroundings for objects. The Arduino sends the distance data (angle and distance) via serial communication to a Python script running on a connected computer. The Python script then uses Pygame to visualize this data as a radar-like display, showing detected objects in real-time.

**Components:**
*   Arduino (e.g., Uno, Nano)
*   Ultrasonic Sensor (e.g., HC-SR04)
*   Servo Motor (continuous rotation or standard, adapted for sweeping)
*   Computer to run Python visualization

**Software & Libraries:**
*   Arduino IDE
*   Python 3
*   Pygame (for visualization)
*   Pyserial (for communication with Arduino)

**Setup & Running:**
1.  **Arduino:**
    *   Connect the ultrasonic sensor (Trig, Echo, VCC, GND) and servo motor (Signal, VCC, GND) to the appropriate pins on the Arduino as defined in `servo_ultasonic_scanner/servo_ultasonic_scanner.ino`.
    *   Upload the `servo_ultasonic_scanner.ino` sketch to your Arduino.
    *   Note the serial port your Arduino is connected to (e.g., `/dev/ttyACM0` on Linux, `COM3` on Windows).
2.  **Python:**
    *   Ensure Python 3, Pygame, and Pyserial are installed. You can typically install them using pip:
        ```bash
        pip install pygame pyserial
        ```
    *   Open `servo_ultasonic_scanner/radar_plot.py`.
    *   Update the `DEFAULT_SERIAL_PORT` variable in the script to match your Arduino's serial port.
    *   Run the script:
        ```bash
        python servo_ultasonic_scanner/radar_plot.py
        ```
    *   A Pygame window should open, displaying the radar visualization.

### 2. Red Dot Tracker

**Description:**
This project uses a camera (e.g., a webcam) and OpenCV to detect a red-colored object (like a laser pointer dot or a red ball). Once detected, the system calculates the object's position relative to the center of the camera's view. This information is then used to control a pan-tilt servo mechanism (simulated or actual) to "follow" or "point at" the red dot. The `tracker_radar.py` script provides a visual interface using Py5 (a Python version of Processing) to display a radar-like view and the camera feed.

The servo control is architected with a client-server model:
*   `tracker_radar.py` acts as the client, sending pitch and yaw commands.
*   `servo_controller/server.py` listens for these commands.
*   `servo_controller/driver.py` (presumably) interfaces with the servo hardware.

**Components:**
*   Camera (e.g., USB webcam)
*   Computer to run Python scripts
*   (Optional/Implied) Pan-tilt servo mechanism controllable by the `servo_controller/driver.py`. This might be Raspberry Pi GPIOs, an Arduino, or another motor controller.

**Software & Libraries:**
*   Python 3
*   OpenCV (`opencv-python`)
*   NumPy
*   Py5
*   Dependencies listed in `red_dot_tracker/requirements.txt` (likely includes the above).

**Setup & Running:**
1.  **Environment:**
    *   Navigate to the `red_dot_tracker/` directory.
    *   It's recommended to use a Python virtual environment.
    *   Install dependencies:
        ```bash
        pip install -r requirements.txt
        ```
        (This should install `opencv-python`, `numpy`, `py5`, and any other specific needs like a library for servo control if `driver.py` uses one not typically bundled.)
2.  **Servo Controller (if applicable):**
    *   The `servo_controller/server.py` needs to be running to receive commands if you have a physical servo setup. This might involve configuring `servo_controller/driver.py` for your specific hardware.
    *   Run the server (likely in a separate terminal):
        ```bash
        python red_dot_tracker/servo_controller/server.py
        ```
3.  **Tracker Application:**
    *   Run the main tracking and visualization script:
        ```bash
        python red_dot_tracker/tracker_radar.py
        ```
    *   A Py5 window should open showing the camera feed, detection status, and radar display. The terminal will show servo commands if a target is detected.
    *   The simpler `red_dot_tracker/tracker.py` can be run independently for basic red dot detection without the Py5 GUI or servo control:
        ```bash
        python red_dot_tracker/tracker.py
        ```

## Future Enhancements
*   Detailed hardware connection diagrams.
*   Troubleshooting common issues.

---
README generated with assistance from an AI pair programmer. 
