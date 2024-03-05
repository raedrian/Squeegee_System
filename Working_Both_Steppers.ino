
// Include the AccelStepper library:
#include "AccelStepper.h"
#include <Servo.h>

// Define stepper motor connections and motor interface type. 
// Motor interface type must be set to 1 when using a driver
#define dirPin 2
#define stepPin 3

#define dirPin2 4
#define stepPin2 5

#define motorInterfaceType 1

#define spr 200 //stepsPerRevolution
#define homePin 10
#define leftButtonPin 7
#define rightButtonPin 8
#define limitSwitchPin 6

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);


// Previous button states
bool prevLeftButtonState = HIGH;
bool prevRightButtonState = HIGH;


//Servo
Servo myservo;
const int servoPin = 9; // PWM pin for the servo
const int posDown = 30;
const int posUp = 100;

void setup() {
  //Serial.begin(9600);
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(500);
  stepper2.setMaxSpeed(500);

  pinMode(homePin, INPUT_PULLUP);   // Enable internal pullup resistor
  pinMode(leftButtonPin, INPUT_PULLUP);   // Enable internal pullup resistor
  pinMode(rightButtonPin, INPUT_PULLUP); // Enable internal pullup resistor
  pinMode(limitSwitchPin, INPUT_PULLUP);  // Enable internal pullup resistor

  
}



void Home() {
    myservo.write(posUp);
    //Serial.println("HOME PIN = ON");
    while (digitalRead(limitSwitchPin) == HIGH) {
      //Serial.println("LIMIT PIN = OFF");
      stepper.setSpeed(500);
      stepper.moveTo(200);

      stepper2.setSpeed(500);
      stepper2.moveTo(200);

      stepper.run();
      stepper2.run();
    }
    Serial.println("LIMIT PIN = ON");
    stepper.setCurrentPosition(0.0);

}



void moveRight() {
  myservo.write(posDown);
  delay(50);
  while (digitalRead(rightButtonPin) == LOW){
    stepper.setSpeed(-500);
    stepper.moveTo(200);

    stepper2.setSpeed(-500);
    stepper2.moveTo(200);
    
    stepper.run();
    stepper2.run();
  }
}

void moveLeft() {
  myservo.write(posDown);
  delay(50);
  while (digitalRead(leftButtonPin) == LOW){
    stepper.setSpeed(500);
    stepper.moveTo(200);

    stepper2.setSpeed(500);
    stepper2.moveTo(200);

    stepper.run();
    stepper2.run();
  }
}

void loop() { 

  // Check button presses and releases
  bool currentLeftButtonState = digitalRead(leftButtonPin);
  bool currentRightButtonState = digitalRead(rightButtonPin);

  if (digitalRead(homePin) == LOW) {
    Home();
  }

  Serial.println("HOME PIN = OFF");
  
  
  // Move gantry left if left button pressed and previously released
  if (currentLeftButtonState == LOW) {
    moveLeft();
  }

  // Move gantry right if right button pressed and previously released
  if (currentRightButtonState == LOW) {
    moveRight();
  }

   // Update previous button states
  prevLeftButtonState = currentLeftButtonState;
  prevRightButtonState = currentRightButtonState;
}





/*
void moveRight() {
 // while (stepper.currentPosition() != -spr*14) //RIGHT
  
    stepper.setSpeed(-500);
    stepper.moveTo(-200*14);
    stepper.runSpeedToPosition();
  
}

void moveLeft() {
 // while (stepper.currentPosition() != spr*14) //LEF
    stepper.setSpeed(500);
    stepper.moveTo(200*14);
    stepper.runSpeedToPosition();
  
}
*/
