/* =============================================================
 * DancingBears
 * Base on Longmont Strip1
 * Runs on 5V Arduino with 16 MHz clock (3.3v uses 8 MHz clock), 
 * Xbee #2
 * 
 *  
 *  LED string on SPI port pins 11 and 13 CLK and MOSI.
 *  Xbee on the UART pins RXI TXD, 57600 baud to match bootloader, 
 *   DIO3 is tied to GRN (reset thru 0.1 uF cap), "digital line passing" 
 *   forwards DIO3 of master, which is tied to DTR so the Arduino IDE 
 *   can reset chip and start programming sequence.  
 *  
 *  2 menu switch inputs on pins 5 and 7, active low
 *  
 *  LED Layout: 86 Leds, only half are on bears, other half are between
 *  
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "Wire.h"             // i2c for accel
#define NUMLEDS 86            // Number of LEDs in strip
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
//float finc = 0.2;      // increment much less than 1.0
float finc = 3.0;      // increment much less than 1.0
float hueBase = 0;
int led = 0;
float index = 0;
float index2 = 0;

//#define MAXMODE 10  
#define MAXMODE 1  
int mode = 0 ;           
int old_mode = 1;

#define MAXLEVEL 9  
int level = 6;
int old_level = 0;

int buttons_in = 3;
int old_buttons = 3;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

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
// This function takes floating point values for 
// hue = h range 0 to 360
// saturation = s range 0 to 1.0
// value = v range 0 to 1.0
// and returns a 32 bit color code for the RGB values 
// needed for the LED string: 0x00GGRRBB
// Use the Adafruit DotStar libraries as much as possible

unsigned long getColor3(float h, float s, float v){
  
  unsigned long color1;
  unsigned long color2;
  uint16_t hue;
  uint8_t sat;
  uint8_t val;

  hue = floor(h*65535.0/360.0);
  sat = floor(s*255);
  val = floor(v*255);
  
  color1= strip.ColorHSV(hue,sat,val);
  color2 = strip.gamma32(color1);

  return(color2);
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
  if(bright<0.003){
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

  // LED strip
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
   
  // Xbee 
//  Serial.begin(19200);   // Serial comms with the Xbee
  Serial.begin(57600);   // Serial comms with the Xbee

  // Debugging
//  Serial.begin(19200);   // Serial comms to the IDE for debug
//  Serial.print("ProMini3: startup Ok");
  pinMode(7,OUTPUT);    // digital flag for timing on scope
  digitalWrite(7,LOW);
    
  // Toggle menu switch on pins 5 and 7
  pinMode(5,INPUT);
  pinMode(7,INPUT);
  
}

//=================================================================================

void loop() {

  // Xbee comms
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    
    //Serial.print(XbeeOut);

    // if(XbeeIn == 1){
    //   tone(8,440,TONETIME);
    //   delay(TONETIME);
    // }

  }


  // Read 4 toggle switch input, normally high, pulled low when toggle is active
  buttons_in = 0;
  if(digitalRead(5)==HIGH) buttons_in +=1;
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
    findex += finc;
    if(findex>360) findex -= 360;
    if(findex<0) findex += 360;
    index = findex;
    
    // level will control brightness and speed of the spiral
    if(level==0) {bright_cmd=0; shift=9; finc=0;}
    if(level==1) {bright_cmd=0.075; shift=8; finc=0.1;}
    if(level==2) {bright_cmd=0.100; shift=7; finc=0.2;}
    if(level==3) {bright_cmd=0.125; shift=6; finc=0.3;}
    if(level==4) {bright_cmd=0.15; shift=5; finc=0.4;}
    if(level==5) {bright_cmd=0.2; shift=4; finc=0.5;}
    if(level==6) {bright_cmd=0.3; shift=3; finc=0.6;}
    if(level==7) {bright_cmd=0.5; shift=2; finc=0.7;}
    if(level==8) {bright_cmd=0.7; shift=1; finc=1.0;}
    if(level==9) {bright_cmd=1.0; shift=0; finc=3.0;}
  
//    if(bright<bright_cmd)bright += 0.0005;   // range 0 to 1.0
//    if(bright>bright_cmd)bright -=0.0005;
    bright=bright_cmd;      // TEST don't ramp      
  }

  // ***************** mode 0, all RED for night light *******************************************************
  if(mode==0){
    
    for(led=0;led<NUMLEDS;led++){
      index2=index-led*1.5;
      while(index2>360)index2-=360;
      while(index2<0)index2+=360;
      color1 = getColor(120,1,bright);        // use base color
      strip.setPixelColor(led,color1);
    }

//    color1 = getColor(120,1,0.1);        // mode marker
//    strip.setPixelColor(0,color1);
}
  // ***************** mode 1, flow *******************************************************
  if(mode==1){
    
    for(led=0;led<NUMLEDS;led++){
      index2=index-led*1.5;
      while(index2>360)index2-=360;
      while(index2<0)index2+=360;
      color1 = scaleColor(getColor3(index2,1,1.0),bright);        // use base color
      strip.setPixelColor(led,color1);
    }
    //color1 = getColor(0,1,0.1);        // mode marker
    //strip.setPixelColor(0,color1);
  }
        
  
  // ************** end of modes ********************************
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs  
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
