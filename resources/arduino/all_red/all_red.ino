
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            5

#define NUMPIXELS 66

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void updateMarker(int initial_idx, int num_leds, unsigned char r, unsigned char g, unsigned char b)
{
  for (int i = initial_idx; i < initial_idx + num_leds; i++)
  {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
}

const int num_markers = 5 ;

int marker_idx[num_markers] = {0,13,26,39,52}; //0,13,26,39,52


void setup()
{
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.
  for (int m=0; m < num_markers; m++)
    updateMarker(marker_idx[m],12,255,0,0);
  
  pixels.show();
}

void loop()
{
}
