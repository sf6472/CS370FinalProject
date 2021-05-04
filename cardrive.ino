/*
 * Some of ideas in the code were orignaly from websites.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include <SharpIR.h>    //**This library was originally from https://github.com/guillaume-rico/SharpIR. We changed the code for reference voltage inside to work with our setup
#define IRFront A1 // define signal pin
#define IRRight A2 // define signal pin
#define IRLeft A3 // define signal pin
#define IRBack A4 // define signal pin

Adafruit_MPU6050 mpu;
SharpIR SharpIRFront(IRFront, 1080);
SharpIR SharpIRLeft(IRLeft, 1080);
SharpIR SharpIRRight(IRRight, 1080);
SharpIR SharpIRBack(IRBack, 1080);

const int FrontGap = 30;
const int BackGap = 30;
const int LeftGap = 30;
const int RightGap = 30;

/*
  Define communication pins with RPi 4
*/
const byte commForward = 34;    //to rpi GPIO17
const byte commBackward = 36;   //to rpi GPIO27
const byte commLeft = 35;       //to rpi GPIO23
const byte commRight = 37;      //to rpi GPIO24

const byte obstacleForward = 40;    //to rpi GPIO5
const byte obstacleBackward = 42;   //to rpi GPIO6
const byte obstacleLeft = 44;       //to rpi GPIO19
const byte obstacleRight = 46;      //to rpi GPIO26

const byte movementPulse = 38;  //to rpi GPIO22
const byte commOpen = 39;       //to rpi GPIO25 (HIGH when Arduino Due powered on)

//declare variables for gyroscope calibration
double min_x, max_x, mid_x;
//declare variables for gyroscope trajectory correction
double headingAngle, tempGyro;
const int correctionStep = 1;
const int correctionSpeed = 210;//220 for 1.2v rechargeable

// Constants for Pins
const int spd = 210;  //220 for 1.2v rechargeable
const int lrspd = 230;  //240 for 1.2v rechargeable

const byte MOTOR_A = 3;  // FL Motor 1 Interrupt Pin 3 - Right Motor
const byte MOTOR_B = 4;  // FR Motor 2 Interrupt Pin 4 - Left Motor
const byte MOTOR_C = 5;  // RL Motor 3 Interrupt Pin 5 - Rear Right Motor
const byte MOTOR_D = 6;  // RR Motor 4 Interrupt Pin 6 - Rear Left Motor

// Integers for Pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;
volatile int counter_C = 0;
volatile int counter_D = 0;

// Motor FL
int enFL = 8;
int in1 = 28;
int in2 = 29;

// Motor FR
int enFR = 10;
int in3 = 30;
int in4 = 31;

// Motor RL
int enRL = 9;
int in1RL = 50;
int in2RL = 51;

// Motor RR
int enRR = 11;
int in3RR = 32;
int in4RR = 33;

// Interrupt Service Routines

// Motor FL Pulse count ISR
void ISR_countA(){
  counter_A++;  // increment Motor FL counter value
  //Serial.println(counter_A);
} 

// Motor FR Pulse count ISR
void ISR_countB(){
  counter_B++;  // increment Motor FR counter value
  //Serial.println(counter_B);
}

// Motor RL Pulse count ISR
void ISR_countC(){
  counter_C++;  // increment Motor FL counter value
  //Serial.println(counter_C);
} 

// Motor RR Pulse count ISR
void ISR_countD(){
  counter_D++;  // increment Motor FR counter value
  //Serial.println(counter_D);
}

// Function to Move Forward
void MoveForward(int steps, int mspeed){
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   counter_C = 0;  //  reset counter C to zero
   counter_D = 0;  //  reset counter D to zero
   
   // Set Motor FL forward
   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW);

   // Set Motor FR forward
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);

   // Set Motor RL forward
   digitalWrite(in1RL, HIGH);
   digitalWrite(in2RL, LOW);

   // Set Motor RR forward
   digitalWrite(in3RR, HIGH);
   digitalWrite(in4RR, LOW);
   
   // Go forward until distance is reached
   
   int FrontDistance = SharpIRFront.distance();

   while (FrontDistance > FrontGap && steps > counter_A && steps > counter_B && steps > counter_C && steps > counter_D){
    updateGyroAngle();

    if (steps > counter_A) {
      analogWrite(enFL, mspeed);
    } else {
      analogWrite(enFL, 0);
    }
    if (steps > counter_B) {
      analogWrite(enFR, mspeed);
    } else {
      analogWrite(enFR, 0);
    }
    if (steps > counter_C) {
      analogWrite(enRL, mspeed);
    } else {
      analogWrite(enRL, 0);
    }
    if (steps > counter_D) {
      analogWrite(enRR, mspeed);
    } else {
      analogWrite(enRR, 0);
    }
//      analogWrite(enFL, mspeed);
//      analogWrite(enFR, mspeed);
//      analogWrite(enRL, mspeed);
//      analogWrite(enRR, mspeed);
   }
    
  // Stop when done
  analogWrite(enFL, 0);
  analogWrite(enFR, 0);
  analogWrite(enRL, 0);
  analogWrite(enRR, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
  counter_C = 0;  //  reset counter C to zero
  counter_D = 0;  //  reset counter D to zero 
}

// Function to Move in Reverse
void MoveReverse(int steps, int mspeed){
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   counter_C = 0;  //  reset counter C to zero
   counter_D = 0;  //  reset counter D to zero
   
   int BackDistance = SharpIRBack.distance();
//   Serial.println(BackDistance);
   
   // Set Motor FL reverse
   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH);
   
   // Set Motor FR reverse
   digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);
   
   // Set Motor RL reverse
   digitalWrite(in1RL, LOW);
   digitalWrite(in2RL, HIGH);
   
   // Set Motor RR reverse
   digitalWrite(in3RR, LOW);
   digitalWrite(in4RR, HIGH);
   
   // Go in reverse until distance is reached
   while (BackDistance > BackGap && steps > counter_A && steps > counter_B && steps > counter_C && steps > counter_D){
    updateGyroAngle();
    if (steps > counter_A) {
      analogWrite(enFL, mspeed);
    } else {
      analogWrite(enFL, 0);
    }
    if (steps > counter_B) {
      analogWrite(enFR, mspeed);
    } else {
      analogWrite(enFR, 0);
    }
    if (steps > counter_C) {
      analogWrite(enRL, mspeed);
    } else {
      analogWrite(enRL, 0);
    }
    if (steps > counter_D) {
      analogWrite(enRR, mspeed);
    } else {
      analogWrite(enRR, 0);
    }
  }
    
  // Stop when done
  analogWrite(enFL, 0);
  analogWrite(enFR, 0);
  analogWrite(enRL, 0);
  analogWrite(enRR, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
  counter_C = 0;  //  reset counter C to zero
  counter_D = 0;  //  reset counter D to zero 
}

// Function to Side way Right
void SidewayRight(int steps, int mspeed){
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   counter_C = 0;  //  reset counter C to zero
   counter_D = 0;  //  reset counter D to zero
   
   // Set Motor FL forward
   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW);

   // Set Motor FR reverse
   digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);

   // Set Motor RL reverse
   digitalWrite(in1RL, LOW);
   digitalWrite(in2RL, HIGH);
   
   // Set Motor RR forward
   digitalWrite(in3RR, HIGH);
   digitalWrite(in4RR, LOW);

   int RightDistance = SharpIRRight.distance();
   // Go until distance is reached
   while (RightDistance > RightGap && steps > counter_A && steps > counter_B && steps > counter_C && steps > counter_D){
    updateGyroAngle();
    if (steps > counter_A) {
      analogWrite(enFL, mspeed);
    } else {
      analogWrite(enFL, 0);
    }
    if (steps > counter_B) {
      analogWrite(enFR, mspeed);
    } else {
      analogWrite(enFR, 0);
    }
    if (steps > counter_C) {
      analogWrite(enRL, mspeed);
    } else {
      analogWrite(enRL, 0);
    }
    if (steps > counter_D) {
      analogWrite(enRR, mspeed);
    } else {
      analogWrite(enRR, 0);
    }
   }
    
  // Stop when done
  analogWrite(enFL, 0);
  analogWrite(enFR, 0);
  analogWrite(enRL, 0);
  analogWrite(enRR, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
  counter_C = 0;  //  reset counter C to zero
  counter_D = 0;  //  reset counter D to zero 

}

// Function to Side way Left
void SidewayLeft(int steps, int mspeed){
   counter_A = 0;  //  reset counter A to zero
   counter_B = 0;  //  reset counter B to zero
   counter_C = 0;  //  reset counter C to zero
   counter_D = 0;  //  reset counter D to zero
   // Set Motor FL reverse
   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH);

   // Set Motor FR forward
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);

   // Set Motor RL forward
   digitalWrite(in1RL, HIGH);
   digitalWrite(in2RL, LOW);
   
   // Set Motor RR reverse
   digitalWrite(in3RR, LOW);
   digitalWrite(in4RR, HIGH);

   int LeftDistance = SharpIRLeft.distance();
   // Go until distance is reached
   while (LeftDistance > LeftGap &&steps > counter_A && steps > counter_B && steps > counter_C && steps > counter_D){
    updateGyroAngle();
    if (steps > counter_A) {
      analogWrite(enFL, mspeed);
    } else {
      analogWrite(enFL, 0);
    }
    if (steps > counter_B) {
      analogWrite(enFR, mspeed);
    } else {
      analogWrite(enFR, 0);
    }
    if (steps > counter_C) {
      analogWrite(enRL, mspeed);
    } else {
      analogWrite(enRL, 0);
    }
    if (steps > counter_D) {
      analogWrite(enRR, mspeed);
    } else {
      analogWrite(enRR, 0);
    }
  }
    
  // Stop when done
  analogWrite(enFL, 0);
  analogWrite(enFR, 0);
  analogWrite(enRL, 0);
  analogWrite(enRR, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero 
  counter_C = 0;  //  reset counter C to zero
  counter_D = 0;  //  reset counter D to zero 

}


void setup(){
  Serial.begin(115200); //initial serial comm for debug
  pinMode(13, OUTPUT);  //onboard LED
  delay(100);
  //=========Initialize gyroscope==========//
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    analogWrite(13, 100);  //if mpu failed lights up
    while (1) {
      delay(10);
    }
  }
  delay(100);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.print("Calibrate in 3 ");
  delay(1000);
  Serial.print("2 ");
  delay(1000);
  Serial.println("1 ");
  delay(1000);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  min_x = max_x = g.gyro.x;
  //Gyroscope Calibration
  double x;
  for (uint16_t sample = 0; sample < 300; sample++) {
    mpu.getEvent(&a, &g, &temp);
    x = g.gyro.x;
    min_x = min(min_x, x);
    max_x = max(max_x, x);
    mid_x = (max_x + min_x) / 2;    //mid x is the offset
    delay(10);
  }
  Serial.println("Done Calibrate!");
  //=======initial comm pins========//
  pinMode(commForward, INPUT);
  pinMode(commBackward, INPUT);
  pinMode(commLeft, INPUT);
  pinMode(commRight, INPUT);
  pinMode(obstacleForward, OUTPUT);
  pinMode(obstacleBackward, OUTPUT);
  pinMode(obstacleLeft, OUTPUT);
  pinMode(obstacleRight, OUTPUT);  
  pinMode(movementPulse, OUTPUT);
  pinMode(commOpen , OUTPUT);
    //initial states are LOW (Pull down). HIGH trigger.
  digitalWrite(commForward, LOW);
  digitalWrite(commBackward, LOW);
  digitalWrite(commLeft, LOW);
  digitalWrite(commRight, LOW);
  digitalWrite(obstacleForward, LOW);
  digitalWrite(obstacleBackward, LOW);
  digitalWrite(obstacleLeft, LOW);
  digitalWrite(obstacleRight, LOW);
  digitalWrite(movementPulse, LOW);
    //HIGH when Arduino Due powered on.
  //=======end initial comm pins========//

  //analogWrite(13, 50);
  pinMode(MOTOR_A, INPUT);
  pinMode(MOTOR_B, INPUT);
  pinMode(MOTOR_C, INPUT);
  pinMode(MOTOR_D, INPUT);
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_B, LOW);
  digitalWrite(MOTOR_C, LOW);
  digitalWrite(MOTOR_D, LOW);

  // Attach the Interrupts to their ISR's
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_C), ISR_countC, RISING);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_D), ISR_countD, RISING);  // Increase counter B when speed sensor pin goes High
  
  digitalWrite(commOpen, HIGH);   //Tell rpi ready after initialize.

} 

void HeadingFix(int csteps, int correctionSpeed){
    //updateGyroAngle();
    counter_C = 0;
    counter_D = 0;
    //if heading is too right
    if(headingAngle <= -0.45){
       // Set Motor RR forward
       digitalWrite(in3RR, HIGH);
       digitalWrite(in4RR, LOW);
       // Set Motor RL Reverse
       digitalWrite(in1RL, LOW);
       digitalWrite(in2RL, HIGH);
       //rotate to fix heading
       while(csteps > counter_D && csteps > counter_C && headingAngle <= -0.45){
        //update gyro
         updateGyroAngle();
         if(csteps > counter_D){
           analogWrite(enRR, correctionSpeed);
         } else {
           analogWrite(enRR, 0);
         }
         if(csteps > counter_C){
           analogWrite(enRL, correctionSpeed);
         } else {
           analogWrite(enRL, 0);
         }
       }
    }
    //if heading is too left
    if(headingAngle >= 0.45){
       // Set Motor RL forward
       digitalWrite(in1RL, HIGH);
       digitalWrite(in2RL, LOW);
       // Set Motor RR reverse
       digitalWrite(in3RR, LOW);
       digitalWrite(in4RR, HIGH);
       //rotate to fix heading
       while(csteps > counter_C && csteps > counter_D && headingAngle >= 0.45){
        //update gyro
        updateGyroAngle();      
        if(csteps > counter_C){
          analogWrite(enRL, correctionSpeed);
        } else {
          analogWrite(enRL, 0);
        }
        if(csteps > counter_D){
          analogWrite(enRR, correctionSpeed);
        } else {
          analogWrite(enRR, 0);
        }
      }
    }
    //updateGyroAngle();
    //stop after the movement
    analogWrite(enRL, 0);
    analogWrite(enRR, 0);
    //clear step counters
    counter_C = 0;
    counter_D = 0;
}
void updateGyroAngle(){
  //update gyro
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  tempGyro = g.gyro.x - mid_x;
  if(abs(tempGyro) >= 0.0019)       //to get rid of noise from gyroscope
    headingAngle += tempGyro;
  Serial.println(headingAngle);
}

void loop(){
  updateGyroAngle();
  HeadingFix(correctionStep, correctionSpeed);
  HeadingFix(correctionStep, correctionSpeed);
  //delay(9);
  //================get distances from obstacle=============//
    int FrontDistance = SharpIRFront.distance();
    int RearDistance = SharpIRBack.distance();
    int LeftDistance = SharpIRLeft.distance();
    int RightDistance = SharpIRRight.distance();
    //front
    if(FrontDistance <= FrontGap){
      digitalWrite(obstacleForward, HIGH);
    }
    if(FrontDistance > FrontGap){
      digitalWrite(obstacleForward, LOW);
    }
    //back
    if(RearDistance <= BackGap){
      digitalWrite(obstacleBackward , HIGH);
    }
    if(RearDistance > BackGap){
      digitalWrite(obstacleBackward , LOW);
    }
    //left
    if(LeftDistance <= LeftGap){
      digitalWrite(obstacleLeft, HIGH);
    }
    if(LeftDistance > LeftGap){
      digitalWrite(obstacleLeft, LOW);
    }
    //right
    if(RightDistance <= RightGap){
      digitalWrite(obstacleRight, HIGH);
    }
    if(RightDistance > RightGap){
      digitalWrite(obstacleRight, LOW);
    }
    
    //=========RPi drive============//
    if(digitalRead(commForward) == HIGH){
      MoveForward(5, spd);  // move Forward 
      updateGyroAngle();
      HeadingFix(correctionStep, correctionSpeed);
      if(FrontDistance > FrontGap){
        digitalWrite(movementPulse, HIGH);  //send a Pulse after moved by a unit
        digitalWrite(movementPulse, LOW);
      }
    }
    if(digitalRead(commBackward) == HIGH){
      MoveReverse(5, spd);  // move backword 
      updateGyroAngle();
      HeadingFix(correctionStep, correctionSpeed);
      if(RearDistance > BackGap){
        digitalWrite(movementPulse, HIGH);  //send a Pulse after moved by a unit
        digitalWrite(movementPulse, LOW);
      }
    }
    if(digitalRead(commLeft) == HIGH){
      SidewayLeft(5, spd);  // move left
      updateGyroAngle();
      HeadingFix(correctionStep, correctionSpeed);
      if(LeftDistance > LeftGap){
        digitalWrite(movementPulse, HIGH);  //send a Pulse after moved by a unit
        digitalWrite(movementPulse, LOW);
      }
    }
    if(digitalRead(commRight) == HIGH){
      SidewayRight(5, spd); // moveright 
      updateGyroAngle();
      HeadingFix(correctionStep, correctionSpeed);
      if(RightDistance > RightGap){
        digitalWrite(movementPulse, HIGH);  //send a Pulse after moved by a unit
        digitalWrite(movementPulse, LOW);
      }
    }
    //=================================//
    //delay(5);  // Wait one second
    //delay(10);
  
}
