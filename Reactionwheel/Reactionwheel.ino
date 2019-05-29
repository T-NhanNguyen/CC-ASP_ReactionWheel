#include <FastLED.h>
#include "MeOrion.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

float wheelmass = 50;   //used for calculation of Inertia
float wheelradius = 50; //used for calculation of Inertia
float cubemass = 20;
float gravity = 9.8;

  //Gyro Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

  //Motor Controller
MeEncoderNew xmotor(0x09, SLOT1); //  Motor at slot1 .  can use the example library in Makeblock to change address
MeEncoderNew zmotor(0x09, SLOT2); //  motor at slot2

  //LEDs
#define NUM_LEDS 5    // How many leds are in the strip?
#define DATA_PIN 5    // Data pin that led data will be written out over
CRGB leds[NUM_LEDS];  // This is an array of leds.  One item for each led in your strip.

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
  xmotor.begin();   //motor1
  zmotor.begin();   //motor2
//------------------------------Setup for LEDs------------------------------
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
}

void loop(){
  //DON'T REMOVE BELOW LINE. This is where you're able to extract angle value.
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //This is the tested range of the each axis. Use this to 'map' your values.
  //x range: 0-360
  //y range: -90-90
  //z range: -180-180
  int val_X = map(euler.x(), 0, 360, -25, 25);  //remap the limits from -10 and 90 to 0 and 5.
  int val_Y = map(euler.y(), -90, 90, -25, 25);  //remap the limits from -10 and 90 to 0 and 5.
  int val_Z = map(euler.z(), -180, 180, -25, 25);  //remap the limits from -10 and 90 to 0 and 5.
  //val_X = val_X*10;
  //val_Y = val_X*10;
  //val_Z = val_Z*10;
  printbno0555(val_X, val_Y, val_Z);
  rotateZ(val_Z);
  rotateX(val_Y);
}


void rotateZ(int i){
  /*Y axis rotation should be passed through
   * positive value is tilting back
   * negative value is tilting forward
   */
  if(i < 0)                     //We want to spin CW so that the angular momentum will push it back
    zmotor.runSpeed(i);   //assumming that motor can handle 10k rpm. will spin accordingly to the magnitude of it's angle
  if(i > 0)                     //We want to spin CCW so that the angular momentum will push it forward
    zmotor.runSpeed(i);
  //Serial.println(i);
}

void rotateX(int i){
  /*Z axis rotation should be passed through
   * positive value is tilting right
   * negative value is tilting left
   */
  if(i < 0)                     //We want to spin CW so that the angular momentum will push it left
    xmotor.runSpeed(i);         //assumming that motor can handle 10k rpm. will spin accordingly to the magnitude of it's angle
  if(i > 0)                     //We want to spin CCW so that the angular momentum will push it right
    xmotor.runSpeed(i);
    
  //Serial.print("pos: ");
  //Serial.println(xmotor.getCurrentPosition());
}
/*int mathangularVelo(float i){
  //I is theta, of an axis
    /*    negative i = ccw. correcting motor should spin cw
    *     positive i = cw. correcting motor should spin ccw
    *
  float angmom = cubemass*gravity;//*cos(abs(i));
  float inertia = wheelmass*wheelradius^2;
  float velocity = angmom-inertia;
  if(i > 0)
    velocity = velocity*(-1);
  return();
}
*/
void printbno0555(int x, int y, int z){
    /* Display the floating point data */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

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











/*    LEDs TESTING  */
void blinktest(){
  for(int i = 0; i < NUM_LEDS; i = i + 1) {
    // Cycling on and off through all the LEDs
    leds[i] = CRGB::Red;
    FastLED.show();   // Make the LED pop up.
    delay(100);
    leds[i] = CRGB::Black;   // Turn our current led back to black for the next loop around
  }
}
void gyrolights(int value){
  leds[value] = CRGB::Orange; //Set the angle value of the BNO055 to the index of array.
  FastLED.show();
  delay(100);
  leds[value-1] = CRGB::Black;
  leds[value+1] = CRGB::Black;
}
