#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

///////////////////////////////////////////////////////////////////////////////
//WS2811 LED control
//WS2811 LEDS
#define WS2811_PIN  9
#define NUM_LEDS 11
#define BRIGHTNESS 50
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, WS2811_PIN, NEO_GRB + NEO_KHZ800);

#define COLOR_NORMAL    strip.Color(0, 255, 0)
#define COLOR_SPEEDUP   strip.Color(150, 0, 100)
#define COLOR_CRAZY     strip.Color(200, 80, 0)
#define COLOR_MAXSPEED  strip.Color(200, 0, 0)
#define COLOR_RC        strip.Color(0, 0, 255)


void WS28xxSetColor(uint32_t c);
void WS28xxSetPixelColor(uint16_t led, uint32_t c);

void rainbowCycle(uint8_t wait);
void rainbow(uint8_t wait);
void colorWipe(uint32_t c, uint8_t wait);
void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength );
uint32_t Wheel(byte WheelPos);

uint8_t red(uint32_t c);
uint8_t green(uint32_t c);
uint8_t blue(uint32_t c);





///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//WS2811

//set all LEDs to one color
void WS28xxSetColor(uint32_t c)
{
  for(uint16_t i=0; i<strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

void WS28xxSetPixelColor(uint16_t led, uint32_t c)
{
  strip.setPixelColor(led, c);
  strip.show();
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for(j=0; j<256 * 5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength )
{

  if(whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;

  int head = whiteLength - 1;
  int tail = 0;

  int loops = 3;
  int loopNum = 0;

  static unsigned long lastTime = 0;


  while(true){
    for(int j=0; j<256; j++) {
      for(uint16_t i=0; i<strip.numPixels(); i++) {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) ){
          strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
        }
        else{
          strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }

      }

      if(millis() - lastTime > whiteSpeed) {
        head++;
        tail++;
        if(head == strip.numPixels()){
          loopNum++;
        }
        lastTime = millis();
      }

      if(loopNum == loops) return;

      head%=strip.numPixels();
      tail%=strip.numPixels();
        strip.show();
        delay(wait);
    }
  }

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3,0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3,0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0,0);
}

uint8_t red(uint32_t c) {
  return (c >> 16);
}
uint8_t green(uint32_t c) {
  return (c >> 8);
}
uint8_t blue(uint32_t c) {
  return (c);
}
