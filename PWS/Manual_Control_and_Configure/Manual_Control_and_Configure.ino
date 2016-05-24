#include <Servo.h>

int value = 0;

Servo firstMotor, secondMotor, thirdMotor, fourthMotor;

// the setup function runs once when you press reset or power the board
void setup() {
  firstMotor.attach(9);
  //secondMotor.attach(6);
  //thirdMotor.attach(10);
  //fourthMotor.attach(11);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  firstMotor.writeMicroseconds(value);
  //secondMotor.writeMicroseconds(value);
  //thirdMotor.writeMicroseconds(value);
  //fourthMotor.writeMicroseconds(value);
  if (Serial.available()) {
    value = Serial.parseInt();
  }
}

