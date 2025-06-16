#include <RF24.h>
#include <Arduino.h> // Ensure constrain() and PI are available


// Define motor control pins
const int stepPin1 = 2;
const int dirPin1 = 3;
const int stepPin2 = 4;
const int dirPin2 = 5;
const int stepPin3 = 8; //diff for mini3
const int dirPin3 = 7;

#define laserPowerPwmPin 6

// Initialize RF24 radio
RF24 radio(10, 9);                // CE, CSN pins
const byte address[6] = "00001";  // Radio pipe address

// Structure to hold incoming command data
struct CommandData {
  float x;      // X-axis movement
  float y;      // Y-axis movement
  float r;      // Rotation
  byte laser;   // Laser state
  byte power;   // Laser power
  float speed;  // Speed factor
};
CommandData receivedData;

// Constants for robot kinematics
const float WHEEL_RADIUS = 0.029;  // in meters
const float ROBOT_RADIUS = 0.161;  // in meters
const float STEPS_PER_REV = 8288;  // steps per revolution

/*
// Assuming receivedData.x, y are in m/s and r is in rad/s
float motor1AngularVelocity = (receivedData.y + ROBOT_RADIUS * receivedData.r) / WHEEL_RADIUS;
float motor2AngularVelocity = (-0.866 * receivedData.x - 0.5 * receivedData.y + ROBOT_RADIUS * receivedData.r) / WHEEL_RADIUS;
float motor3AngularVelocity = (0.866 * receivedData.x - 0.5 * receivedData.y + ROBOT_RADIUS * receivedData.r) / WHEEL_RADIUS;

float motor1SPS = motor1AngularVelocity * (STEPS_PER_REV / (2.0 * 3.1459));
float motor2SPS = motor2AngularVelocity * (STEPS_PER_REV / (2.0 * 3.1459));
float motor3SPS = motor3AngularVelocity * (STEPS_PER_REV / (2.0 * 3.1459));

// Calculate individual step delays for each motor
// Handle division by zero for stationary motors
int speed1 = (motor1SPS == 0) ? 0 : (unsigned long)(1000000.0 / abs(motor1SPS));
int speed2 = (motor2SPS == 0) ? 0 : (unsigned long)(1000000.0 / abs(motor2SPS));
int speed3 = (motor3SPS == 0) ? 0 : (unsigned long)(1000000.0 / abs(motor3SPS));
*/
unsigned long lastStepTime1;
unsigned long lastStepTime2;
unsigned long lastStepTime3;

// Global declarations (or outside setup/loop)
unsigned long speed1 = 0; // Initialize to 0 or MAX_ACCEPTABLE_DELAY
unsigned long speed2 = 0;
unsigned long speed3 = 0;

// A delay of 0 or a very small number could mean max speed.
// Consider adding a minimum delay to prevent motors from attempting
// impossible speeds or locking up.
const unsigned long MIN_STEP_DELAY = 500; // e.g., 500 us for 2000 SPS
const unsigned long MAX_STEP_DELAY = 50000; // e.g., 50000 us for 20 SPS (very slow)

/*
if (speed1 != 0) speed1 = constrain(speed1, MIN_STEP_DELAY, MAX_STEP_DELAY);
if (speed2 != 0) speed2 = constrain(speed2, MIN_STEP_DELAY, MAX_STEP_DELAY);
if (speed3 != 0) speed3 = constrain(speed3, MIN_STEP_DELAY, MAX_STEP_DELAY);
*/

/*
// In loop(), set directions based on the sign of angular velocity
digitalWrite(dirPin1, motor1AngularVelocity >= 0 ? HIGH : LOW);
digitalWrite(dirPin2, motor2AngularVelocity >= 0 ? HIGH : LOW);
digitalWrite(dirPin3, motor3AngularVelocity >= 0 ? HIGH : LOW);
*/

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize radio communication
  if (!radio.begin()) {
    Serial.println("Radio hardware is not responding!");
    while (1) {}  // Halt execution
  }
  radio.openReadingPipe(0, address);
  radio.startListening();

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(laserPowerPwmPin, OUTPUT);

  Serial.println("Robot is ready.");
}

void moveMotors() {
    unsigned long currentMicros = micros();

    // Motor 1
    if (speed1 > 0 && currentMicros - lastStepTime1 >= speed1) { // speed1 is the delay now
        digitalWrite(stepPin1, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin1, LOW);
        lastStepTime1 = currentMicros;
    }

    // Motor 2
    if (speed2 > 0 && currentMicros - lastStepTime2 >= speed2) {
        digitalWrite(stepPin2, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin2, LOW);
        lastStepTime2 = currentMicros;
    }

    // Motor 3
    if (speed3 > 0 && currentMicros - lastStepTime3 >= speed3) {
        digitalWrite(stepPin3, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin3, LOW);
        lastStepTime3 = currentMicros;
    }
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(CommandData));

    // Assuming x, y, r from receivedData are velocities (e.g., m/s, rad/s)
    float scaled_x = receivedData.x;
    float scaled_y = receivedData.y;
    float scaled_r = receivedData.r; // If r is already rad/s, no need to multiply by 5

    // Kinematics to find *angular velocities* of each wheel (rad/s)
    float motor1AngularVelocity = (scaled_y + ROBOT_RADIUS * scaled_r) / WHEEL_RADIUS;
    float motor2AngularVelocity = (-0.866 * scaled_x - 0.5 * scaled_y + ROBOT_RADIUS * scaled_r) / WHEEL_RADIUS;
    float motor3AngularVelocity = (0.866 * scaled_x - 0.5 * scaled_y + ROBOT_RADIUS * scaled_r) / WHEEL_RADIUS;

    // Convert angular velocities to Steps Per Second (SPS)
    // Ensure 2.0 * PI for float calculation
    float motor1SPS = motor1AngularVelocity * (STEPS_PER_REV / (2.0 * PI));
    float motor2SPS = motor2AngularVelocity * (STEPS_PER_REV / (2.0 * PI));
    float motor3SPS = motor3AngularVelocity * (STEPS_PER_REV / (2.0 * PI));

    // Set directions based on the sign of angular velocity
    digitalWrite(dirPin1, motor1AngularVelocity >= 0 ? HIGH : LOW);
    digitalWrite(dirPin2, motor2AngularVelocity >= 0 ? HIGH : LOW);
    digitalWrite(dirPin3, motor3AngularVelocity >= 0 ? HIGH : LOW);

    // Convert SPS to microseconds delay between steps.
    // If SPS is 0 (motor should be stopped), set delay to 0 or a very large number.
    // A value of 0 for speedX in moveMotors() means don't step.
    speed1 = (abs(motor1SPS) < 0.1) ? 0 : (unsigned long)(1000000.0 / fabs(motor1SPS));
    speed2 = (abs(motor2SPS) < 0.1) ? 0 : (unsigned long)(1000000.0 / fabs(motor2SPS));
    speed3 = (abs(motor3SPS) < 0.1) ? 0 : (unsigned long)(1000000.0 / fabs(motor3SPS));

    // Add constraints to prevent excessively high/low speeds
    const unsigned long MIN_ACCEPTABLE_DELAY = 500; // Smallest delay (fastest speed, e.g., 2000 SPS)
    const unsigned long MAX_ACCEPTABLE_DELAY = 50000; // Largest delay (slowest speed, e.g., 20 SPS)

    if (speed1 != 0) speed1 = constrain(speed1, MIN_ACCEPTABLE_DELAY, MAX_ACCEPTABLE_DELAY);
    if (speed2 != 0) speed2 = constrain(speed2, MIN_ACCEPTABLE_DELAY, MAX_ACCEPTABLE_DELAY);
    if (speed3 != 0) speed3 = constrain(speed3, MIN_ACCEPTABLE_DELAY, MAX_ACCEPTABLE_DELAY);


    // Serial prints for debugging
    Serial.print("Ang Vel (rad/s): ");
    Serial.print(motor1AngularVelocity); Serial.print(" ");
    Serial.print(motor2AngularVelocity); Serial.print(" ");
    Serial.println(motor3AngularVelocity);

    Serial.print("SPS: ");
    Serial.print(motor1SPS); Serial.print(" ");
    Serial.print(motor2SPS); Serial.print(" ");
    Serial.println(motor3SPS);

    Serial.print("Step Delays (us): ");
    Serial.print(speed1); Serial.print(" ");
    Serial.print(speed2); Serial.print(" ");
    Serial.println(speed3);
    Serial.println(""); // Blank line for readability

    Serial.print("Laser: ");
    Serial.print(receivedData.laser);
    Serial.print(" Power: ");
    Serial.println(receivedData.power);

    if (receivedData.laser == 1) {
      analogWrite(laserPowerPwmPin, 255 - receivedData.power);
    } else {
      analogWrite(laserPowerPwmPin, 255);
    }
  }
  // Always call moveMotors; it will handle whether to step based on 'speedX' values
  moveMotors();
}

/*
float speed_scale = receivedData.speed / 255.0; // Assuming 0-255
if (speed_scale < 0.01) speed_scale = 0.01; // Prevent full stop due to very small scale

float scaled_x = receivedData.x * speed_scale;
float scaled_y = receivedData.y * speed_scale;
float scaled_r = receivedData.r * speed_scale; // If r is also a velocity command
*/
