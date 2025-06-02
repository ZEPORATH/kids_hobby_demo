#include <Servo.h>

// === Configuration ===
const int SERVO_PIN = 9;      // Pin for continuous rotation servo
const int TRIG_PIN = 10;      // Ultrasonic sensor Trigger
const int ECHO_PIN = 11;      // Ultrasonic sensor Echo

const int STEP_DELAY = 20;    // Delay between steps in ms
const int MAX_DISTANCE_CM = 200;

Servo radarServo;

int sweepDirection = 1;       // 1 = CW, -1 = CCW
unsigned long lastStepTime = 0;
int stepCount = 0;
int maxSteps = 70;            // Total steps per sweep direction (adjust per speed & time)

// --- Setup ---
void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Radar (360° Servo) Initializing...");

  radarServo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopServo();
  delay(1000);

  Serial.println("Radar Ready. Format: Step,Distance(cm)");
}

// --- Main Loop ---
void loop() {
  unsigned long now = millis();

  if (now - lastStepTime >= STEP_DELAY) {
    lastStepTime = now;

    // Move servo in current direction
    moveServo(sweepDirection * 0.5);  // Moderate speed
    stepCount++;

    // Measure and print distance
    long dist = getDistanceCm();
    Serial.print(sweepDirection == 1 ? stepCount : (maxSteps - stepCount));
    Serial.print(",");
    Serial.println(dist);
    Serial.flush();
    if (stepCount >= maxSteps) {
      sweepDirection *= -1;  // Reverse direction
      stepCount = 0;
      stopServo();
      delay(300);  // Pause before reversing
    }
  }
}

// --- Get Distance from HC-SR04 ---
long getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, MAX_DISTANCE_CM * 2 * 29.1);
  long distance = duration / 2 / 29.1;

  if (distance == 0 || distance > MAX_DISTANCE_CM) {
    return MAX_DISTANCE_CM;
  } else {
    return distance;
  }
}

// --- Control Continuous Rotation Servo ---
void moveServo(float speed) {
  // speed ∈ [-1.0, 1.0]
  // -1.0 = full CW, 0.0 = stop, 1.0 = full CCW
  speed = constrain(speed, -1.0, 1.0);
  int pulse = 1500 + speed * 250;  // 1000–2000µs
  radarServo.writeMicroseconds(pulse);
}

void stopServo() {
  radarServo.writeMicroseconds(1500);  // Stop command
}
