#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int numServos = 5;
const int servoPins[numServos] = {0, 1, 2, 3, 4};
const int servoMin = 150; // Minimum servo pulse length
const int ServoMid = 300;
const int servoMax = 600; // Maximum servo pulse length

// Servo limits
const int servoLimits[numServos][2] = {{0, 180}, {10, 170}, {0, 180}, {0, 180}, {30, 150}}; // Adjust limits for each servo

const char *servoNames[numServos] = {"Servo Base", "Servo Swing", "Servo Arm One", "Servo Arm Two", "Servo Gripper"};

// Servo positions for smooth control
int currentServoPos[numServos] = {90, 90, 90, 90, 90}; // Initial positions

// Link lengths (assume two-link planar arm for simplicity)
float link1 = 10.0; // Length of arm link 1
float link2 = 10.0; // Length of arm link 2

// Speed control variable
int speedDelay = 10; // Speed control (lower is faster)

// Function to smoothly move a servo
void moveServoSmooth(int servoNum, int targetPos)
{
  int currentPos = currentServoPos[servoNum];

  // Ensure target position is within servo limits
  targetPos = constrain(targetPos, servoLimits[servoNum][0], servoLimits[servoNum][1]);

  // Move the servo gradually towards the target position
  while (currentPos != targetPos)
  {
    if (currentPos < targetPos)
    {
      currentPos++;
    }
    else if (currentPos > targetPos)
    {
      currentPos--;
    }
    int pulse = map(currentPos, 0, 180, servoMin, servoMax);
    pwm.setPWM(servoPins[servoNum], 0, pulse);
    currentServoPos[servoNum] = currentPos; // Update current position
    delay(speedDelay);                      // Control speed of movement
  }
}

// Inverse Kinematics for planar arm
void inverseKinematics(float x, float y)
{
  float d = sqrt(x * x + y * y); // Distance from base to end-effector
  if (d > (link1 + link2) || d < fabs(link1 - link2))
  {
    Serial.println("Target unreachable"); // Safety check: out of reach
    return;
  }

  float angle2 = acos((x * x + y * y - link1 * link1 - link2 * link2) / (2 * link1 * link2)); // Elbow angle
  float angle1 = atan2(y, x) - atan2(link2 * sin(angle2), link1 + link2 * cos(angle2));       // Shoulder angle

  // Convert angles to degrees
  int servoAngle1 = (int)(angle1 * 180.0 / PI);
  int servoAngle2 = (int)(angle2 * 180.0 / PI);

  // Move servos (Servo Arm One and Servo Arm Two) to new positions
  moveServoSmooth(2, servoAngle1);
  moveServoSmooth(3, servoAngle2);
}

// Safety mechanism to prevent out-of-bounds or dangerous movements
bool isSafePosition(float x, float y)
{
  // Add safety zones where the arm is restricted
  if (x < 0 || y < 0 || x > (link1 + link2) || y > 10)
  { // Example soft limits
    Serial.println("Unsafe position!");
    return false;
  }
  return true;
}

void setup()
{
  Serial.begin(115200); // Set up Serial communication at 115200 baud rate
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  Serial.println("Ready to receive commands. Use format: S<num> <angle>");
  pwm.setPWM(servoPins[0 - 4], 0, ServoMid); // Initialize servos at mid position
}

void loop()
{
  // Read serial input to control the servos
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("S"))
    {
      int servoNum = input.substring(1, 2).toInt();
      int servoValue = input.substring(3).toInt();

      if (servoNum >= 0 && servoNum < numServos)
      {
        moveServoSmooth(servoNum, servoValue);
        Serial.print("Moving Servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.println(servoValue);
      }
      else
      {
        Serial.println("Invalid servo number.");
      }
    }
    else
    {
      Serial.println("Invalid command. Use format: S<num> <angle>");
    }
  }
}