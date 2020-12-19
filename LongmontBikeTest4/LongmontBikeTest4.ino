/* =============================================================
 * Longmont Bike 4
 * Run this on the breadboard, 7 leds, Xbee #2 
 * based on BB7 snow globe code
 *  
 *  LED string on SPI port pins 11 and 13 CLK and MOSI, pin 10 SPISEL 
 *   is an enable signal to the voltage translator TXB0104, must be high
 *   to send data to LEDs
 *  Accelerometer on SPI bus, pins 11,12,13, uses pin 10 SPISEL as 
 *   an active low select.
 *  Xbee on the UART pins RXI TXD, 57600 baud to match bootloader, 
 *   DIO3 is tied to GRN (reset thru 0.1 uF cap), "digital line passing" 
 *   forwards DIO3 of master, which is tied to DTR so the Arduino IDE 
 *   can reset chip and start programming sequence.  
 *  Piezo for tone on pin 8.
 *  Max4466 Mic Amp on A0 analog input for sound.
 *  
 *  2 menu switch inputs on pins 4 and 7.
 *  
 *  LED Layout: 2 sets 0 to 20, and 21 to 41, plus one extra at 42.
 *  
 *  Dr Tom Flint, 27 Jan 2019
 *  Rework for the 3.3 volt ProMini, 22 Feb 2019
 *  Add over-the-air setup, 1 Mar 2019
 *  Adapt for Globe BB-6a, 2 April 2019
 *  From BB7 hack a new Bike Lights version, 13 Nov 2020
 *  Add daisy-chain dotstar on end of fiber box, 22 Nov 2020
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel
//#define NUMLEDS 7            // Number of LEDs in strip
#define NUMLEDS 250            // Number of LEDs in strip
#define TONETIME 200          // mSec for tone outputs

Adafruit_DotStar strip = Adafruit_DotStar(NUMLEDS, DOTSTAR_BRG);

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

// Colors
unsigned long color1;   // hue (HSV) 
float bright = 0.0;      
float bright_cmd = 0.0; 
float bright2 = 0.0;
unsigned int shift = 0;

float findex = 120;       // Keep track of hue with a float, limit loop delay
float finc = 0.2;      // increment much less than 1.0
float hueBase = 0;
int led = 0;
int index = 0;
int index2 = 0;

#define NUMHUES 7    //   red    ora    yel  grn   cyan   indigo  violet   repeat red 
double Hue[NUMHUES+1] = { 120,   115,   90,  4,    340,   260,    210,      120 };
double HueDelta = 5;

#define NUMSHADES 20
unsigned long colorA[NUMHUES * NUMSHADES];
int index3 = 0;   // indices 3,4 for new color methods
int index4 = 0;
int delay2 = 0;

//#define MAXMODE 10  
#define MAXMODE 1  
int mode = 0;           
int old_mode = 0;

#define MAXLEVEL 9  
int level = 2;
int old_level = 0;

int buttons_in = 3;
int old_buttons = 3;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

// Accelerometer
LIS3DH a1( SPI_MODE, 10 );  // Use spi with pin10 as chip select
float ax, ay, az;     // accelerometer readings
float mx, my, mz;     // magnitude and limit to 0 to 1.0


//=================================================================================
// rgb values are doubles on scale 0 to 1.0

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

//=================================================================================
// This function converts HSV to RGB triplets

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
// This function takes floating point values for 
// hue = h range 0 to 360
// saturation = s range 0 to 1.0
// value = v range 0 to 1.0
// and returns a 32 bit color code for the RGB values 
// needed for the LED string: 0x00GGRRBB
unsigned long getColor2(float h, float s, float v){
  
  unsigned long temp;
  
  colorIn.h = h;
  colorIn.s = s;
  colorIn.v = v;
  colorOut = hsv2rgb(colorIn);
  
  // compensation for low end output
  if(colorOut.r>0.005) colorOut.r = colorOut.r + 0.01*(1-colorOut.r)*(1-colorOut.r);
  if(colorOut.g>0.005) colorOut.g = colorOut.g + 0.01*(1-colorOut.g)*(1-colorOut.g);
  if(colorOut.b>0.005) colorOut.b = colorOut.b + 0.01*(1-colorOut.b)*(1-colorOut.b);

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
// Scale a color that is already in the unsigned long format by a floating point 
// value to achieve dimming from a full brightness reference
unsigned long scaleColor(unsigned long color, float bright){

  unsigned long scolor = 0;
  unsigned long red = 0;
  unsigned long grn = 0;
  unsigned long blu = 0;

  // divide the unsigned long into 3 fields
  blu=color & 0x000000FF;
  grn=(color & 0x0000FF00)>>8;
  red=(color & 0x00FF0000)>>16;

  // compensation for non-linear output
  if(bright<0){
    bright=0;
  }else{
    bright = bright*bright+0.001;
  }
  if(bright>1)bright=1;
  
  // scale each color 
  blu = bright * (float)blu;
  grn = bright * (float)grn;
  red = bright * (float)red;

  // assemble back into an unsigned long
  scolor = red;
  scolor = scolor<<8;
  scolor = scolor + grn;
  scolor = scolor<<8;
  scolor = scolor + blu;
  
  return(scolor);
}

//=================================================================================
// Scale a color that is already in the unsigned long format by shifting
// value to achieve dimming from a full brightness reference
unsigned long scaleColor2(unsigned long color, int shift){

  unsigned long scolor = 0;
  unsigned long red = 0;
  unsigned long grn = 0;
  unsigned long blu = 0;

  if(shift<0)return(0L);
  if(shift>7)return(0L);

  // divide the unsigned long into 3 fields
  blu=color & 0x000000FF;
  grn=(color & 0x0000FF00)>>8;
  red=(color & 0x00FF0000)>>16;

  // scale each color 
  blu = blu>>shift;
  grn = grn>>shift;
  red = red>>shift;

  // assemble back into an unsigned long
  scolor = red;
  scolor = scolor<<8;
  scolor = scolor + grn;
  scolor = scolor<<8;
  scolor = scolor + blu;
  
  return(scolor);
}
//=================================================================================

void setup() {

  // Pin 10 is the SPISEL line, set it HIGH to use the LEDs or the Piezo, both are enabled
  // thru the 3.3 to 5 volt level shifter.  Pin 10 LOW to use spi with the accelerometer.
  pinMode(10,OUTPUT);
  digitalWrite(10,LOW);

  // Startup tone
  digitalWrite(10,HIGH);
  tone(8,440,TONETIME);
  delay(TONETIME);
  tone(8,880,TONETIME);
  delay(TONETIME);
  tone(8,1760,TONETIME);
  delay(TONETIME);
  digitalWrite(10,LOW);
  
  // LED strip
  digitalWrite(10,HIGH);
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
  digitalWrite(10,LOW);
   
  // Xbee 
//  Serial.begin(19200);   // Serial comms with the Xbee
  Serial.begin(57600);   // Serial comms with the Xbee

  // Debugging
//  Serial.begin(19200);   // Serial comms to the IDE for debug
//  Serial.print("ProMini3: startup Ok");
  pinMode(7,OUTPUT);    // digital flag for timing on scope
  digitalWrite(7,LOW);
  
  // Accelerometer
  a1.settings.adcEnabled = 0;
  a1.settings.tempEnabled = 0;
  a1.settings.accelSampleRate = 25;  // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  a1.settings.accelRange = 4;        // Max G force readable.  Can be: 2, 4, 8, 16
  a1.settings.xAccelEnabled = 1;
  a1.settings.yAccelEnabled = 1;
  a1.settings.zAccelEnabled = 1;
  a1.begin();
  
  // Toggle menu switch on pins 4 and 7
  pinMode(4,INPUT);
  pinMode(7,INPUT);

  // setup colors
  int hi; // hue index
  int ci; // color index
  int di; // delta index
  double base;
  double delta;
  ci=0;
  for(hi=0;hi<NUMHUES;hi++){          // loop over the hues
    base = Hue[hi];
    delta = Hue[hi]-Hue[hi+1];
    if(delta<0) delta+=360;
    delta = delta / NUMSHADES;
    for(di=0;di<NUMSHADES;di++) {       // generate shades of each hue
      index2 = base - delta*di;
      if(index2>360)index2-=360;
      if(index2<0)index2+=360;
      colorA[ci]=getColor(index2,1,1);  // full brightness
      ci++;
    }
  }
}

//=================================================================================

void loop() {

  // Read the 3d accelerometer on the SPI
  digitalWrite(4,HIGH);
  ax = a1.readFloatAccelX();
  ay = a1.readFloatAccelY();
  az = a1.readFloatAccelZ();
  digitalWrite(4,LOW);

  // Xbee comms
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    
    //Serial.print(XbeeOut);

    if(XbeeIn == 1){
      tone(8,440,TONETIME);
      delay(TONETIME);
    }
    if(XbeeIn == 2){
      tone(8,880,TONETIME);
      delay(TONETIME);
    }
    if(XbeeIn == 3){
      tone(8,1760,TONETIME);
      delay(TONETIME);
    }

  }


  // Read 4 toggle switch input, normally high, pulled low when toggle is active
  buttons_in = 0;
  if(digitalRead(4)==HIGH) buttons_in +=1;
  if(digitalRead(7)==HIGH) buttons_in +=2;
  if(buttons_in != old_buttons){
    // got a new button press
    old_buttons = buttons_in;

    if(buttons_in == 1){
      mode +=1;
      if(mode>MAXMODE) mode=0;
    }
    if(buttons_in == 2){
      level +=1;
      if(level>MAXLEVEL) level=0;
    }
  }


  // ******** mode ALL *******************************************************    
  if(mode >= 0){
    // update the hue index, first as float then as an int
    findex -= finc;
    if(findex>360) findex -= 360;
    if(findex<0) findex += 360;
    index = findex;
    
    // level will control brightness and speed of the spiral
    if(level==0) {bright_cmd=0; shift=9;}
    if(level==1) {bright_cmd=0.05; shift=8;}
    if(level==2) {bright_cmd=0.1; shift=7;}
    if(level==3) {bright_cmd=0.15; shift=6;}
    if(level==4) {bright_cmd=0.2; shift=5;}
    if(level==5) {bright_cmd=0.3; shift=4;}
    if(level==6) {bright_cmd=0.4; shift=3;}
    if(level==7) {bright_cmd=0.6; shift=2;}
    if(level==8) {bright_cmd=0.8; shift=1;}
    if(level==9) {bright_cmd=1.0; shift=0;}
  
//    if(bright<bright_cmd)bright += 0.0005;   // range 0 to 1.0
//    if(bright>bright_cmd)bright -=0.0005;
    bright=bright_cmd;      // TEST don't ramp      
  }

  // ***************** mode 0, flow *******************************************************
  if(mode==0){
    
    for(led=0;led<NUMLEDS;led++){
      index2=led*1+index;
      if(index2>360)index2-=360;
      if(index2<0)index2+=360;
      color1 = getColor(index2,1,bright);        // use base color
      strip.setPixelColor(led,color1);
    }
  }
  // ***************** mode 1, flow *******************************************************
  if(mode==1){
    
    for(led=0;led<NUMLEDS;led++){
      index2=led*1+index;
      if(index2>360)index2-=360;
      if(index2<0)index2+=360;
      color1 = getColor2(index2,1,bright);        // use base color
      strip.setPixelColor(led,color1);
    }
  }
        
  // ***************** mode -1, rainbow flow *******************************************************
  if(mode==-1){
    strip.clear();                       

    // only change the index slowly
    if(delay2<200){
      delay2++;
    }else{
      delay2=0;
      if(index3<NUMHUES*NUMSHADES){
        index3++;
      }else{
        index3=0;
      }
    }
      
    // update the strip all passes
    for(led=0;led<NUMLEDS;led++){
      index4 = index3+led;    
      if(index4>=NUMHUES*NUMSHADES) index4 -= NUMHUES*NUMSHADES;
//      strip.setPixelColor(led,colorA[index4]);
      strip.setPixelColor(led, scaleColor(colorA[index4],bright) );
    }
  }

  // ***************** mode 1, Show 7 Colors **************************
  if(mode==-3){
    strip.clear();

    for(led=0;led<NUMLEDS;led++){
      strip.setPixelColor(led, getColor(Hue[led],1,bright) );
    }
  }

  // ***************** mode 2, Scaled 7 Colors *************************
  if(mode==2){
    strip.clear();

    for(led=0;led<NUMLEDS;led++){
//      strip.setPixelColor(led, getColor(Hue[led],1,bright) );
//      strip.setPixelColor(led, colorA[led*NUMSHADES] );
      strip.setPixelColor(led, scaleColor(colorA[led*NUMSHADES],bright) );
    }
  }

  // ***************** mode 3, Shift 7 Colors *************************
  if(mode==3){
    strip.clear();

    for(led=0;led<NUMLEDS;led++){
//      strip.setPixelColor(led, getColor(Hue[led],1,bright) );
//      strip.setPixelColor(led, colorA[led*NUMSHADES] );
//      strip.setPixelColor(led, scaleColor(colorA[led*NUMSHADES],bright) );
      strip.setPixelColor(led, scaleColor2(colorA[led*NUMSHADES],shift) );
    }
  }
  
  // ***************** mode 4:10, Show individual colors with delta  ****
  if(mode>3){
    strip.clear();

    for(led=0;led<NUMLEDS;led++){
      index2=Hue[mode-4]-3*HueDelta +led*HueDelta;
      strip.setPixelColor(led, getColor(index2,1,bright) );
    }
  }
    
  
  // ************** end of modes ********************************
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
