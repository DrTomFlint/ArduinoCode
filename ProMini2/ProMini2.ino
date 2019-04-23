#include <Adafruit_DotStar.h>
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 6 // Number of LEDs in strip
#define DELAY 10    // used at end of loop()
#define TONETIME 300  // mSec for tone outputs
//#define DEBUG 1

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
float hue[6] = {120, 105, 60, 0, 240, 180};
float hueinc[6];
int huestep = 600;
int hues = 6;
int huehold = 1000;
int holdi = 0;
int huei=0;
int stepi=0;
int index = 0;
int index2 = 0;
float offset = 6;
int mode = 1;
unsigned long color1;   // hue (HSV) 
float s1 = 1.0;         // saturation (HSV)
float bright = 0.5;     // value (HSV)

int redx, greenx, bluex;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

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

  // Startup tone
  tone(2,440,TONETIME);
  delay(TONETIME);
  tone(2,880,TONETIME);
  delay(TONETIME);
  tone(2,1760,TONETIME);
  delay(TONETIME);
/*
  delay(TONETIME*3);

  tone(2,440,TONETIME);
  delay(TONETIME);
  tone(2,220,TONETIME);
  delay(TONETIME);
*/
  // LED strip
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
   
  hueinc[0] = ( hue[1] - hue[0] ) / huestep;
  hueinc[1] = ( hue[2] - hue[1] ) / huestep;
  hueinc[2] = ( hue[3] - hue[2] ) / huestep;
  hueinc[3] = ( hue[4] - 360) / huestep;
  hueinc[4] = ( hue[5] - hue[4] ) / huestep;
  hueinc[5] = ( hue[0] - hue[5] ) / huestep;

  // Xbee 
  Serial.begin(19200);   // Serial comms with the Xbee

}

//=================================================================================

void loop() {

  if(mode==1){
    stepi++;
    if(stepi>=huestep){
      stepi=0;
      huei++;
      if(huei>=hues){
        huei=0;
      }
      mode=2;
    }
  }else{
    holdi++;
    if(holdi>=huehold){
      holdi=0;
      mode=1;
    }
  }

  index = hue[huei]+hueinc[huei]*stepi;
  if(index>=360) index -= 360;
  if(index<0) index += 360;
  
  for(led=0;led<12;led++){
//   index2 = index - offset*led*hueinc[huei];
   index2 = index + offset*led;
    while(index2>=360) index2 -= 360;
    while(index2<0) index2 += 360;
    
    color1 = getColor( index2, s1, bright );
    strip.setPixelColor(led,color1);
  }

  strip.show();                     // Refresh strip

  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    Serial.print(XbeeOut);
    //Serial.write(XbeeOut);
    XbeeCount++;
    if(XbeeIn>=0){
      if(XbeeIn<=5){
        huei = XbeeIn;
        holdi = 0;
        stepi = 0;
        mode = 1;
      }
    }
  }
  
  delay(DELAY);                        // Variable delay controls apparent speed

}

//=================================================================================
