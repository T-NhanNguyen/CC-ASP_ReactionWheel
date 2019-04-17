#include <FastLED.h>
#include <Wire.h>
#include "MeOrion.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
	//Gyro Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);
	//Motor Controller
MeEncoderNew motor1(0x09, SLOT1);	//  Motor at slot1
MeEncoderNew motor2(0x09, SLOT2); //  motor at slot2
#define M1speed;
#define M2speed;
	//LEDs
#define NUM_LEDS 5    // How many leds are in the strip?
#define DATA_PIN 5    // Data pin that led data will be written out over
CRGB leds[NUM_LEDS];  // This is an array of leds.  One item for each led in your strip.

void setup(){
	Serial.begin(9600);
	delay(2000);	//safety purposes for quick reaction disengage
//------------------------------Setup for BNO055------------------------------
	if(!bno.begin()){
	/* check your connections, there was an error in the connection */
	Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	while(1);
	bno.setExtCrystalUse(true);
	}
//------------------------------Setup for Motor Controller------------------------------
	motor1.begin();
	motor2.begin();
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
	int val = map(euler.y(), -10, 90, 0, 5);	//remap the limits from -10 and 90 to 0 and 5.

	Serial.println(val);		//For testing purposes.
	if(euler.y() > 0){
		//blinktest();
		gyrolights(val);

	}
}

/*		LEDs TESTING	*/
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
	leds[value] = CRGB::Orange;	//Set the angle value of the BNO055 to the index of array.
	FastLED.show();
	delay(100);
	leds[value-1] = CRGB::Black;
	leds[value+1] = CRGB::Black;
}