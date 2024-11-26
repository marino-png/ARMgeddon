/*servo motor driver board control
   Home Page
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 102
#define servoMAX 500

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(60);

  srituhobby.setPWM(0, 0, servoMIN);
  srituhobby.setPWM(1, 0, servoMIN);
  srituhobby.setPWM(2, 0, servoMIN);
  srituhobby.setPWM(3, 0, servoMIN);
  srituhobby.setPWM(4, 0, servoMIN);
}

void loop() {
  for(int servoNumber = 0; servoNumber < 6;servoNumber++){
    //Serial.println(servoNumber);
    if(servoNumber == 1){
      //do nothing
    }else{
      for (int angle = servoMIN ; angle <servoMAX; angle += 15 ){
        Serial.print(servoNumber);
        Serial.print("     ");
        int degreeAngle = map(angle, servoMIN,servoMAX,0,180);
        Serial.println(degreeAngle);
        srituhobby.setPWM(servoNumber, 0, angle);
        delay(100);
      }
       for (int angle = servoMAX ; angle > servoMIN; angle -= 15 ){
        Serial.print(servoNumber);
        Serial.print("     ");
        int degreeAngle = map(angle, servoMIN,servoMAX,0,180);
        Serial.println(degreeAngle);
        srituhobby.setPWM(servoNumber, 0, angle);
        delay(100);
      }
    }
     Serial.println("next servo");
  }
  // for (int servo = 0; servo < 4; servo++ ) {
  //   srituhobby.setPWM(servo, 0, servoMIN);
  //   Serial.println(servo);
  //   delay(300);
  // }

  // for (int servo = 3; servo >= 0; servo-- ) {
  //   srituhobby.setPWM(servo, 0, servoMAX);
  //   Serial.println(servo);
  //   delay(300);
  // }
}