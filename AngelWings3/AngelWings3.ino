/* =============================================================
 * AngelWings2
 * 
 * Adapted for Xbee #5, used to be globe with single halo loop.
 * Modified from the Longmont long-strip tests
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
 *  2 menu switch inputs on pins 4 and 7.  Pins 5 and 6 for second switch.
 *  
 *  
 *  LED Layout: 2 sets 0 to 20, and 21 to 41, plus one extra at 42.
 *  
 *  Dr Tom Flint, 27 Jan 2019
 *  Rework for the 3.3 volt ProMini, 22 Feb 2019
 *  Add over-the-air setup, 1 Mar 2019
 *  Adapt for Globe BB-6a, 2 April 2019
 *  From BB7 hack a new Bike Lights version, 13 Nov 2020
 *  Add daisy-chain dotstar on end of fiber box, 22 Nov 2020
 *  Alter for long strip tests, 21 Dec 2020
 *  Modify for use with the Angel Wings, 15 June 2023
 *  
 *  AngelWings2 cuts all non-running code from AngelWings1
 *  AngelWings3 setup defaults with just 10 leds connected
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel
#define NUMLEDS 17            // Number of LEDs in strip
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

float findex = 120;       // Keep track of hue with a float, limit loop delay
float finc = 3.0;      // increment much less than 1.0
int led = 0;
int lit = 0;
float index = 0;
float index2 = 0;


int index3 = 0;   // indices 3,4 for new color methods
int index4 = 0;
int delay2 = 0;

#define MAXMODE 4  
int mode = 1;           
int old_mode = 0;

#define MAXLEVEL 2  
int level = 2;
int old_level = 0;

int buttons_in = 0;
int old_buttons = 0;

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
  
  // upper menu switch on pins 4 and 5
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  // lower menu switch on pins 6 and 7
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  // Pin 13 is onboard LED?
  pinMode(13,OUTPUT);
    
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
  //  4 red     upper switch down     mode -
  //  5 blue    upper switch up       mode +
  //  6 white   lower switch down     bright -
  //  7 yellow  lower switch up       bright +
  buttons_in = 0;
  if(digitalRead(4)==LOW) buttons_in +=1;
  if(digitalRead(5)==LOW) buttons_in +=2;
  if(digitalRead(6)==LOW) buttons_in +=4;
  if(digitalRead(7)==LOW) buttons_in +=8;
  if(buttons_in != old_buttons){
    // got a new button press
    old_buttons = buttons_in;

    if(buttons_in == 1){
      mode -=1;
      if(mode<0) mode=MAXMODE;
    }
    if(buttons_in == 2){
      mode +=1;
      if(mode>MAXMODE) mode=0;
    }
    if(buttons_in == 4){
      level -=1;
      if(level<0) level=MAXLEVEL;
    }
    if(buttons_in == 8){
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
    
    // level will control brightness
    if(level==0) bright_cmd=0.3;
    if(level==1) bright_cmd=0.6;
    if(level==2) bright_cmd=1.0;
    
    bright=bright_cmd;  

  
  }

  // ***************** mode 0, all RED for night light *******************************************************
  if(mode==0){
    
    finc = 0.1;

    for(led=0;led<NUMLEDS;led++){
      if(led<10){
        color1 = getColor3(120,1,bright);      
      }else{
        color1 = 0;
      }
      strip.setPixelColor(led,color1);
    }

  }
  // ***************** mode 1, flow down *******************************************************
  if(mode==1){
    
    finc = 0.1;

    for(led=0;led<NUMLEDS;led++){
      // only first 10 leds have optical fibers on them
      if(led<10){
        index2=index+led*7;
        while(index2>360)index2-=360;
        while(index2<0)index2+=360;
        color1 = getColor3(index2,1,bright);       
      }else{
        // extra leds should be off
        color1 = 0;
      }
      strip.setPixelColor(led,color1);
    }
  }

  // ***************** mode 2, flow up *******************************************************
  if(mode==2){
    
    finc = 0.1;

    for(led=0;led<NUMLEDS;led++){
      // only first 10 leds have optical fibers on them
      if(led<10){
        index2=index-led*7;
        while(index2>360)index2-=360;
        while(index2<0)index2+=360;
        color1 = getColor3(index2,1,bright);       
      }else{
        // extra leds should be off
        color1 = 0;
      }
      strip.setPixelColor(led,color1);
    }
  }

  // ***************** mode 3, flow down with single fiber lit *******************************************************
  if(mode==3){

    finc = 0.0333;

    index2++;
    if(index2>100){
      index2=0;
      lit++;
      if(lit>9)lit=0;
    }
    for(led=0;led<NUMLEDS;led++){
      // only 1 led is turned on
      if(led==lit){
        color1 = getColor3(index,1,bright);       
      }else{
        // extra leds should be off
        color1 = 0;
      }
      strip.setPixelColor(led,color1);
    }
  }

  // ***************** mode 4, flow up with single fiber lit *******************************************************
  if(mode==4){

    finc = 0.0333;

    index2++;
    if(index2>100){
      index2=0;
      lit--;
      if(lit<0)lit=9;
    }
    for(led=0;led<NUMLEDS;led++){
      // only 1 led is turned on
      if(led==lit){
        color1 = getColor3(index,1,bright);       
      }else{
        // extra leds should be off
        color1 = 0;
      }
      strip.setPixelColor(led,color1);
    }
  }

  // ************** end of modes ********************************
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs  
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
