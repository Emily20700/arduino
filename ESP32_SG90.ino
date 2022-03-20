#include <ESP32Servo.h>

Servo myservo; // create servo object to control, a servo
// 16 servo objects can be created on the ESP32
int pos = 0; // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: O(used by on-board button),1-17,18(used by on-board 
#if defined(ARDUINO_ESP32S2_DEV)
int servoPin = 17; 
#else
int servoPin = 18; 
#endif

const byte interruptPin = 0;


void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rotate, CHANGE);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  // using default mm/max of l000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void loop() {
  
  
  myservo.write(pos);
  delay(15);

  
}

void rotate() {
  if (digitalRead(inPin)==1)
    pos = pos +30;
  

  if (pos >=180) pos = 0;

}