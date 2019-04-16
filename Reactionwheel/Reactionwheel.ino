#include <bitswap.h>
#include <chipsets.h>
#include <color.h>
#include <colorpalettes.h>
#include <colorutils.h>
#include <controller.h>
#include <cpp_compat.h>
#include <dmx.h>
#include <FastLED.h>
#include <fastled_config.h>
#include <fastled_delay.h>
#include <fastled_progmem.h>
#include <fastpin.h>
#include <fastspi.h>
#include <fastspi_bitbang.h>
#include <fastspi_dma.h>
#include <fastspi_nop.h>
#include <fastspi_ref.h>
#include <fastspi_types.h>
#include <hsv2rgb.h>
#include <led_sysdefs.h>
#include <lib8tion.h>
#include <noise.h>
#include <pixelset.h>
#include <pixeltypes.h>
#include <platforms.h>
#include <power_mgt.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define LED_PIN     2
#define NUM_LEDS    5
#define BRIGHTNESS  64
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

void FillLEDsFromPaletteColors();
void turnLEDoff();

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  //Initialise the sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //LEDs
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);

  bno.setExtCrystalUse(true);

  Serial.println("INITIATING");
}

void loop(void){
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 if(euler.x() > 50){
  Serial.println("beep beep");
  FillLEDsFromPaletteColors();
 }
 else
  turnLEDoff();
}

void FillLEDsFromPaletteColors()
{
    for(int i = 0; i < NUM_LEDS; i = i + 1) {
      // Turn our current led on to white, then show the leds
      leds[i] = CRGB::Yellow;

      // Show the leds (only one of which is set to white, from above)
      FastLED.show();

      // Wait a little bit
      delay(100);

      // Turn our current led back to black for the next loop around
      leds[i] = CRGB::Black;
   }
}
void turnLEDoff()
{
    for(int i = 0; i < NUM_LEDS; i = i + 1) {
      // Turn our current led back to black for the next loop around
      leds[i] = CRGB::Black;
      FastLED.show();
   }
}
