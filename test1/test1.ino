#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float link1 = 5.0; // Length of arm link 
float link2 = 5.0;
float link3 = 5.0;

int speedDelay = 10; 

const int servoMin = 150; // Minimum servo pulse length
const int ServoMid = 300;
const int servoMax = 600; // Maximum servo pulse length

const int baseServo =0;// name all the servo motor
const int link1Servo =1;
const int link2Servo =2;
const int link3Servo =3;
const int gripperServo =4;

const int numServos = 5;
const int servoPins[numServos] = {0, 1, 2, 3, 4};

String userInput;

int sliderID;
int valueSlider;

float valueOfX;
float valueOfY;

// Servo positions for smooth control
int currentServoPos[numServos] = {90, 90, 90, 90, 90}; // Initial positions


void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  pwm.setPWM(servoPins[0 - 4], 0, ServoMid); // Initialize servos at mid position
}

void loop(){
  if(Serial.available()>0){
    userInput = Serial.readStringUntil('\n');
    parseMessage(userInput);

    selectMovment(sliderID , valueSlider );

    inverseKinematics(valueOfX, valueOfY);
  }
}

void parseMessage(String userInput){
  int delimiterIndex = userInput.indexOf(':');

  String sliderIDStr = userInput.substring(0, delimiterIndex);
  String valueStr = userInput.substring(delimiterIndex + 1);

  sliderID = sliderIDStr.toInt(); // Convert slider ID to integer
  valueSlider = valueStr.toInt();
}

void selectMovment(int sliderID, int valueSlider){
  if(sliderID == 0){
    moveServoSmooth(0, valueSlider);
  }
  else if(sliderID == 1){
    valueOfX = valueSlider;
  }
  else if(sliderID == 2){
    valueOfY = valueSlider;
  }
}

// Function to smoothly move a servo
void moveServoSmooth(int servoNum, int targetPos){
  int currentPos = currentServoPos[servoNum];
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

void inverseKinematics(float x, float y)
{
  float d = sqrt(x * x + y * y); // Distance from base to end-effector
  if (d > (link1 + link2) || d < fabs(link1 - link2))
  {
    //Starget unreachable 
    return;
  }
  
  float alfa = acos((link1*link1+link2*link2-d*d)/(2*link1*link2));
  float angle2 = PI - alfa; // Elbow angle
  float angle1 = atan2(y, x) - atan2(link2 * sin(alfa), link1 + link2 * cos(alfa));       // Shoulder angle
  float angle3 = atan2(y, x) - angle1 - angle2;

  // Convert angles to degrees
  int servoAngle1 = (int)(angle1 * 180.0 / PI);
  int servoAngle2 = (int)(angle2 * 180.0 / PI);
  int servoAngle3 = (int)(angle3 * 180.0 / PI);

  // Move servos (Servo Arm One and Servo Arm Two) to new positions
  moveServoSmooth(1, servoAngle1);
  moveServoSmooth(2, servoAngle2);
  moveServoSmooth(3, servoAngle3);
}