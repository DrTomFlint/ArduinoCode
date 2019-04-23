// Adafruit DotStar - Version: 1.0.4
#include <Adafruit_DotStar.h>

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

// 12-28-2018 cut off 18 leds from strip
#define NUMPIXELS 5 // Number of LEDs in strip
#define FIBERS 6
#define DELAY 20      // Delay slows down the traveling
#define OFFSET 10   // color offset in degrees out of 360

//#define DEBUG 1

float s1 = 1.0;
float v1 = 0.1;
unsigned long color1 = 0;

float bright = 0.9; // range 0 to 1.0
float speed = 15;   // higher number is longer delay
float shift = 1;   // higher number is faster change colors

float color0 = 0; // base color value
float color[FIBERS];  // color hue values for each fiber
float color0inc = 30;   // shift value for base color
unsigned long colorX;

// My strip is Grn Red Blu order of bits.

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

rgb colorOut;
hsv colorIn;


//----------------------------------------------------------------------------------------

static rgb   hsv2rgb(hsv in);
rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

//----------------------------------------------------------------------------------------
// This function takes floating point values for 
// hue = h range 0 to 360
// saturation = s range 0 to 1.0
// value = v range 0 to 1.0
// and returns a 32 bit color code for the RGB values 
// needed for the LED string: 0x00GGRRBB
unsigned long getColor(float h, float s, float v){
  
  unsigned long temp;
  
  colorIn.h = h;
  colorIn.s = s;
  colorIn.v = v;
  colorOut = hsv2rgb(colorIn);
  colorOut.r *= 255;
  colorOut.g *= 255;
  colorOut.b *= 255;

  temp = colorOut.r;
  temp = temp<<8;
  temp = temp + colorOut.g;
  temp = temp<<8;
  temp = temp + colorOut.b;
  
  return(temp);
}

//----------------------------------------------------------------------------------------

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

 strip.begin(); // Initialize pins for output
 strip.show();  // Turn all LEDs off ASAP
 

}

//----------------------------------------------------------------------------------------

void loop() {
  int i;    
  
    // increment all the color values
    for (i=0;i<FIBERS+1;i++){
      color[i] += shift;
      if(color[i]>360) color[i] -= 360;
    }
    
  // set the fibers to the selected colors
  colorX = getColor(color[0],s1,bright);
  strip.setPixelColor(0, colorX);
  
  colorX = getColor(color[1],s1,bright);
  strip.setPixelColor(1, colorX);
  
  colorX = getColor(color[2],s1,bright);
  strip.setPixelColor(2, colorX);
  
  colorX = getColor(color[3],s1,bright);
  strip.setPixelColor(3, colorX);

  colorX = getColor(color[4],s1,bright);
  strip.setPixelColor(4, colorX);
  
  colorX = getColor(color[0],s1,bright);
  strip.setPixelColor(5, colorX);

  // blackout some of the pixels
  colorX = 0;
//  strip.setPixelColor(0, colorX);
  strip.setPixelColor(1, colorX);
  strip.setPixelColor(2, colorX);
  strip.setPixelColor(3, colorX);
  strip.setPixelColor(4, colorX);
  strip.setPixelColor(5, colorX);
  
  // update the strip
  strip.show();                     // Refresh strip
  delay(speed);                     // Variable delay controls apparent speed

}
