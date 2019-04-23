/* =================================================================
 *  BikeLightsStP copy from ColorTest2, bike lights box, a 5v 
 *  arduino uno, SPI direct into the LEDs, no other sensors, no 
 *  menu buttons.
 *  
 *  HACK FOR ALL GREEN
 */

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
#define DELAY 200      // Delay slows down the traveling
//#define DEBUG 1

// My strip is Grn Red Blu order of bits.

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);  // Setup the LCD object and assign pins
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

// New for ColorTest1:
int led = 0;
int base = 0;
//float hue[6] = {120, 105, 60, 0, 240, 180};
float hue[3] = {0,40,200};
float hueinc[3];
int huestep = 600;
int hues = 3;
int huehold = 1000;
int holdi = 0;
int huei=0;
int stepi=0;
int index = 50;
int index2 = 0;
float offset = 3;
int mode = 1;

int redx, greenx, bluex;

#define F 11
//int fiber[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
//int fiber[12] =   {F, F, F, F, F, F, F, F, F, F,  F,  F};
//int fiber[12] =   {10, 0, 9, 2, 1, 11,   7, 5, 8, 3,  4,  6};
int fiber[12] =   {10, 7, 0, 5, 9, 8,  2, 3,  1, 4,  11, 6};

// older
float h1index = 0;
float h1step = 0.05;
float s1 = 1.0;
float v1 = 0.1;
unsigned long color1 = 0;
int head  = 0, tail = -LENGTH; // Index of first 'on' and 'off' pixels

float h2index = 60;
float h2step = 1.0;
float s2 = 1.0;
float v2 = 0.1;
unsigned long color2 = 0;
int head2  = 0, tail2 = - LENGTH2; // Index of first 'on' and 'off' pixels

// Menu items
#define NMENU 3
int menui = 0;

#define NBRIGHT 6
float brightTab[NBRIGHT]={0.01, 0.05, 0.1, 0.2, 0.5, 1.0};
int brighti = 5;
float bright;

#define NSPEED 6
float speedTab[NSPEED]={50, 25, 10, 5, 2, 1};
int speedi = 5;
float speed;

#define NSHIFT 5
float shiftTab[NSHIFT]={0.1, 0.2, 0.5, 1.0, 2.0};
int shifti = 2;
float shift;

int buttons_in = 0;
int buttons_out = 0;
int old_buttons = 88;

int menuOn = 1;
#define MENU_MILLIS 5000
unsigned long button_last = 0;
unsigned long button_delta = 0;

//=================================================================================

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


//=================================================================================
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


//=================================================================================

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

 bright = brightTab[brighti];
 speed = speedTab[speedi];
 shift = shiftTab[shifti];
 

}


//=================================================================================

void loop() {

  int led2;
  int x;

 
  color1 = getColor(0,1,0.7);
  
  for(led=0;led<12;led++){
    strip.setPixelColor(led,color1);
  }
  if(random(16)==1){
    color1 = getColor(0,1,1);
    strip.setPixelColor(random(12),color1);
  }
  
  strip.show();                     // Refresh strip
  delay(DELAY);                        // Variable delay controls apparent speed

}

//=================================================================================
