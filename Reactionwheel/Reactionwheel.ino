#include "MeOrion.h"            //Library for the Motor driver
#include <SoftwareSerial.h>     //Library for the Motor driver
#include <Adafruit_Sensor.h>    //Library for the BNO055
#include <Adafruit_BNO055.h>    //Library for the BNO055
#include <math.h>
#include <Wire.h>

  //Gyro Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

  //Motor Controller
MeEncoderNew ymotor(0x09, SLOT1); //  Motor at slot1 .  can use the example library in Makeblock to change address
MeEncoderNew zmotor(0x09, SLOT2); //  motor at slot2

  //Photohoresistors for sensor. TopLeft = pr1, TR = pr2, BL = pr3, BR = pr4
int pr1 = A0;
int pr2 = A1;
int pr3 = A2;
int pr4 = A3;
int tol = A6;   //Potentiometer to adjust the sensitivity to the ambient lighting

void setup(){
  Serial.begin(9600);
  delay(1000);  //safety purposes for quick reaction disengage
//------------------------------Setup for BNO055------------------------------
  if(!bno.begin()){
  /* check your connections, there was an error in the connection */
  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  while(1);
  bno.setExtCrystalUse(true);
  }
/*  X axis spins along the global y (top or bottom) axis
 *  Y axis spins along the global x (sides) axis
 *  Z acis spins along the global z (outward or inward) axis
 */
//------------------------------Setup for Motor Controller------------------------------
  ymotor.begin();   //motor1
  zmotor.begin();   //motor2
//------------------------------Setup for PhotoResistor------------------------------
  pinMode(pr1, INPUT);
  pinMode(pr2, INPUT);
  pinMode(pr3, INPUT);
  pinMode(pr4, INPUT);
  pinMode(tol, INPUT);
}

void loop(){
  //This is the main function. This recalibrate all the sensors to have the same range.
  //This is intended to be very intuitive, so that if you want to test anything, you can-
  //do so without rewriting the whole code.
  int pr1val = map(analogRead(pr1), 0, 1024, 0, 100); //800 is max value
  int pr2val = map(analogRead(pr2), 0, 1024, 0, 100); //840 is max value
  int pr3val = map(analogRead(pr3), 0, 1024, 0, 100); //870 is max value
  int pr4val = map(analogRead(pr4), 0, 1024, 0, 100); //870 is max value
  int tolval = map(analogRead(tol), 0, 1024, 0, 100);

  sensorreading(pr1val, pr2val, pr3val, pr4val, tolval);
}

void rotateY(int i){
  //This is to spin motor 1.
  i = i*10;   //Factors of the angular vecloity
  ymotor.runSpeed(i);
}

void rotateZ(int i){
  //This is to spin motor 2.
  i = i*10;   //Factors of the angular velocity
  zmotor.runSpeed(i);
}

int avg(int x, int y){
  //finding the averages of the 2 values.
  int total = (x+y)/2;
  return (total);
}

int absdiff(int x, int y){  
  //Obtainign the absolute value of two values.
  int diff = x-y;
  int absolutediff = abs(diff);
  return absolutediff;
}

void sensorreading(int pr1, int pr2, int pr3, int pr4, int tolval){
  //The core function that takes in the signals and activate the motors accordingly
  if(absdiff(avg(pr1, pr2), avg(pr3, pr4)) > tolval){  //Check the sensitivity of top and bottom
    if(avg(pr1, pr2) > avg(pr3, pr4)){  //if top is greater than bottom
      rotateY(absdiff(avg(pr1, pr2), avg(pr3, pr4)));
      Serial.println("Up");
    }
    else if(avg(pr1, pr2) < avg(pr3, pr4)){   //if bottom is greater than top
      rotateY(absdiff(avg(pr1, pr2), avg(pr3, pr4))*(-1));
      Serial.println("Down");
    }
  }

  if(absdiff(avg(pr1, pr3), avg(pr2, pr4)) > tolval){   //Check the sensitivity of left and right
    if(avg(pr1, pr3) > avg(pr2, pr4)){    //If left is greater than right
      rotateZ(absdiff(avg(pr1, pr3), avg(pr2, pr4)));
      Serial.println("Left");
    }
    else if(avg(pr1, pr3) < avg(pr2, pr4)){   //If right is greater than left
      rotateZ(absdiff(avg(pr1, pr3), avg(pr2, pr4))*(-1));
      Serial.println("Right");
    }
  }
}

void printbno0555(int x, int y, int z){
    /* Display the floating point data */
    //USED FOR DEBUGGING THE GYRO.
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.print("\t\t");
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
  delay(100);
}
