#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN    16
// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 21
unsigned char brightness = 30;
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

unsigned long pixelPrevious = 0;        // Previous Pixel Millis
unsigned long patternPrevious = 0;      // Previous Pattern Millis
int           patternCurrent = 0;       // Current Pattern Number
int           pixelInterval = 50;       // Pixel Interval (ms)
int           pixelQueue = 0;           // Pattern Pixel Queue
int           pixelCycle = 0;           // Pattern Pixel Cycle
uint16_t      pixelCurrent = 0;         // Pattern Current Pixel Number
uint16_t      pixelNumber = LED_COUNT;  // Total Number of Pixels

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
void colorWipe(uint32_t color) {
  strip.setPixelColor(pixelCurrent, color); //  Set pixel's color (in RAM)
  strip.show();                             //  Update strip to match
  pixelCurrent++;                           //  Advance current pixel
  if(pixelCurrent >= pixelNumber)           //  Loop the pattern from the first LED
    pixelCurrent = 0;
}
// Fill the dots one after the other with a color
void colorWipe_delay(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
void strip1() {
  pixelInterval = 100;   
  int i = 0;
  strip.fill(strip.Color(0, 0, 0));
  for(i;i<5;i++)
  {
      strip.setPixelColor((i+pixelNumber-pixelCycle)%pixelNumber, Wheel((i*10 + pixelQueue) & 255)); 
      strip.setPixelColor((i+pixelNumber-pixelCycle+11)%pixelNumber, Wheel((i*10 + pixelQueue+128) & 255)); 
  }
  strip.show();
  pixelCycle++;
  if(pixelCycle>=pixelNumber)
      pixelCycle = 0;
  pixelQueue++;                             //  Advance current cycle
  if(pixelQueue >= 256)
    pixelQueue = 0;                         //  Loop the cycle back to the begining
    
}
void strip2() {
  int i = 0;
  strip.fill(strip.Color(0, 0, 0));
  for(i;i<5;i++)
  {

      int j =(4-i)%7*(150/5)+1;
      
      strip.setPixelColor((i+pixelNumber-pixelCycle)%pixelNumber, strip.Color(j,j,j)); 
      strip.setPixelColor((i+pixelNumber-pixelCycle+11)%pixelNumber, strip.Color(j,j,j)); 
  }
  strip.show();
  pixelCycle++;
  if(pixelCycle>=pixelNumber)
      pixelCycle = 0;
}
void strip3() {
  int i = 0;
  strip.fill(strip.Color(0, 0, 0));
  for(i;i<5;i++)
  {
      int j =i%7*(150/5)+1;
      strip.setPixelColor((i+pixelCycle)%pixelNumber, strip.Color(j,j,j)); 
      strip.setPixelColor((i+pixelCycle+11)%pixelNumber, strip.Color(j,j,j)); 
  }
  strip.show();
  pixelCycle++;
  if(pixelCycle>=pixelNumber)
      pixelCycle = 0;
}
void rainbow1() {          
  pixelInterval = 30;   
  for(uint16_t i=0; i < pixelNumber; i++) {
    strip.setPixelColor(i, Wheel((i + pixelCycle) & 255)); //  Update delay time  
  }
  strip.show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}
void rainbow2()
{
  pixelInterval = 30;
    pixelCycle +=256; 
    strip.rainbow(pixelCycle);
    strip.show(); // Update strip with new contents
    if(pixelCycle >= 5*65536)
      pixelCycle = 0;
}
void pulse_rainbow1()
{
  pixelInterval = 30;
  pixelQueue+=1;
  if(pixelQueue>=brightness*2)
   pixelQueue = 0;
  if(pixelQueue<brightness)
  strip.setBrightness(pixelQueue); // Set BRIGHTNESS to about 1/5 (max = 255)
  else
  strip.setBrightness(brightness+brightness-pixelQueue); // Set BRIGHTNESS to about 1/5 (max = 255)
  pixelCycle +=256; 
    strip.rainbow(pixelCycle);
    strip.show(); // Update strip with new contents
    if(pixelCycle >= 5*65536)
      pixelCycle = 0;
}
void rgb_off()
{
  pixelInterval = 100;
  strip.fill(strip.Color(0, 0, 0));
  strip.show(); // Update strip with new contents
}
