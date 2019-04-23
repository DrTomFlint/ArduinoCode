// Adafruit DotStar - Version: 1.0.4
#include <Adafruit_DotStar.h>

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 43 // for 30 LED / M older string

#define LENGTH 42      // Number of LED in second chase
#define LENGTH2 6      // Number of LED in lead chase
#define DELAY 20      // Delay slows down the traveling
//#define DEBUG 1

// My strip is Grn Red Blu order of bits.

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

float h1index = 0;
float h1step = 17;
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

float bright=0.1;
float speed = 80;
float shift=10;


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

rgb colorOut;
hsv colorIn;

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


void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

 head2 = head2 + NUMPIXELS/4;
 tail2 = tail2 + NUMPIXELS/4;
 

}

void loop() {
    
  // increment the floating point Hue index, range is 0 to 360
  h1index += shift;
  if(h1index>360) h1index -= 360;
  if(h1index<0) h1index += 360;
  color1 = getColor(h1index,s1,bright);

/*
  h2index += shift;
  if(h2index>360) h2index -= 360;
  if(h2index<0) h2index += 360;
  color2 = getColor(h2index,s2,bright);
*/  
  
  strip.setPixelColor(head, color1); // 'On' pixel at head
  strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
//  strip.setPixelColor(head2, color2); // 'On' pixel at head
//  strip.setPixelColor(tail2, 0);     // 'Off' pixel at tail

  strip.show();                     // Refresh strip
  delay(speed);                        // Variable delay controls apparent speed

  if(++head >= NUMPIXELS) head = 0;
//  if(++head2 >= NUMPIXELS) head2 = 0;
  if(++tail >= NUMPIXELS) tail = 0; // Increment, reset tail index
//  if(++tail2 >= NUMPIXELS) tail2 = 0; // Increment, reset tail index
  
}
