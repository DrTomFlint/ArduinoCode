// Adafruit DotStar - Version: 1.0.4
#include <Adafruit_DotStar.h>
// LiquidCrystal - Version: 1.0.7
#include <LiquidCrystal.h>

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

// 12-28-2018 cut off 18 leds from strip
#define NUMPIXELS 12 // Number of LEDs in strip
#define LENGTH 4      // Number of LED in second chase
#define LENGTH2 12      // Number of LED in lead chase
#define DELAY 20      // Delay slows down the traveling
#define OFFSET 10   // color offset in degrees out of 360

//#define DEBUG 1

float h1index = 0;
float h2index = 0;
float h1step = 0.05;
float s1 = 1.0;
float v1 = 0.1;
unsigned long color1 = 0;
int head  = 0, tail = -LENGTH; // Index of first 'on' and 'off' pixels

float bright = 1; // range 0 to 1.0
float speed = 50;   // higher number is longer delay
float shift = 1;   // higher number is faster change colors

#define FIBERS 12
int i,j,k;        // loop counters
float inc0 = 0.3;  // base increment value
float inc[FIBERS]; // increment values for each fiber
float color0 = 0; // base color value
float color[FIBERS];  // color hue values for each fiber
float color0inc = 30;   // shift value for base color
int mode = 1;   // mode 1 is all run, mode 2 is base shift
unsigned long colorX;

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

  color[0] = 0;
  color[1] = 40;
  color[2] = 340;
  
  // Setup the color increment values for each of the fibers
  inc[0] = ( color[1]-color[2] ) / 100;
  inc[1] = ( color[2]-color[2] ) / 100;
  inc[2] = ( color[2]-color[0] ) / 100;

  
}

//----------------------------------------------------------------------------------------

void loop() {
  
  
  if(mode==1){
    
    // increment all the color values
    for (i=0;i<FIBERS;i++){
      color[i] += inc[i];
      if(color[i]>360) color[i] -= 360;
    }
    

  }
  
  
  
/*  
  // set the fibers to the selected colors
  colorX = getColor(color[0],s1,bright);
  strip.setPixelColor(0, colorX);
  strip.setPixelColor(5, colorX);
  
  colorX = getColor(color[1],s1,bright);
  strip.setPixelColor(1, colorX);
  strip.setPixelColor(4, colorX);
  
  colorX = getColor(color[2],s1,bright);
  strip.setPixelColor(2, colorX);
  strip.setPixelColor(3, colorX);
  
  colorX = getColor(color[3],s1,bright);
  strip.setPixelColor(8, colorX);
  strip.setPixelColor(9, colorX);

  colorX = getColor(color[4],s1,bright);
  strip.setPixelColor(7, colorX);
  strip.setPixelColor(10, colorX);
  
  colorX = getColor(color[5],s1,bright);
  strip.setPixelColor(6, colorX);
  strip.setPixelColor(11, colorX);
*/

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

  colorX = getColor(color[5],s1,bright);
  strip.setPixelColor(5, colorX);

  colorX = 0;
  strip.setPixelColor(6, colorX);
  strip.setPixelColor(7, colorX);
  strip.setPixelColor(8, colorX);
  strip.setPixelColor(9, colorX);
  strip.setPixelColor(10, colorX);
  strip.setPixelColor(11, colorX);
  
  // update the strip
  strip.show();                     // Refresh strip
  delay(speed);                     // Variable delay controls apparent speed

}
