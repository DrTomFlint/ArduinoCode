/* =============================================================
 *  Globes1
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
 *  Analog output on pin 9 to a regular LED for beat detection.
 *  2 menu switch inputs on pins 4 and 7.
 *  
 *  LED Layout:
 *  Strip A, 0 to 16, #7 is the center, 0 and 16 face in
 *  Strip B, 17 to 33, #24 is center and should be off, 17 and 34 face in
 *  Strip C, 34 to 50, #41 is center and should be off, 35 and 52 face in
 *  Fibers, 51 and 52, 51 is the right side 
 *  
 *  Dr Tom Flint, 27 Jan 2019
 *  Rework for the 3.3 volt ProMini, 22 Feb 2019
 *  Add over-the-air setup, 1 Mar 2019
 *  Adapt for Globe BB-6a, 2 April 2019
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel
#define NUMPIXELS 54 // Number of LEDs in strip
#define DELAY 2    // used at end of loop()
#define TONETIME 200  // mSec for tone outputs

//Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_RGB);
//Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

// Tests for colors from Octave
int xred = 255;
int xgreen = 0;
int xblue = 255;

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
float findex = 0;       // Keep track of hue with a float, limit loop delay
float finc = 0.05;      // increment much less than 1.0
float hueBase = 0;
int led = 0;
int index = 0;
int index2 = 0;
float offset = 20;
unsigned long color1;   // hue (HSV) 
float s1 = 1.0;         // saturation (HSV)
float bright = 0.1;
int mode = 2;
int modei = 0;

// add a moving sprite
int sprite = 3;
float fsprite = 3;
float spriteInc = 0.1;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

// Accelerometer
//LIS3DH a1( I2C_MODE, 0x19 );  // Use I2C and default sparkfun breakout address = 0x19
LIS3DH a1( SPI_MODE, 10 );  // Use spi with pin10 as chip select
float ax, ay, az;     // accelerometer readings
float mx, my, mz;     // magnitude and limit to 0 to 1.0

// Beat detector
#define SAMPLEPERIODUS 200
float sample, value, envelope, beat, thresh;
unsigned char i;
int aread;
int duty;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

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
// This function 

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


//================================================================================
// This function takes floating point values from the accelerometer
// and returns a 32 bit color code for the RGB values 
// needed for the LED string: 0x00GGRRBB
// This method does not ensure fully saturated colors
unsigned long getColor2(float x, float y, float z){

  #define CMULT (float)120;
  #define COFF 10;
  unsigned long temp;
  
  colorOut.r = abs(x)*CMULT+COFF;
  if(colorOut.r<0)colorOut.r=0;
  if(colorOut.r>255)colorOut.r=255;
  colorOut.g = abs(y)*CMULT+COFF;
  if(colorOut.g<0)colorOut.g=0;
  if(colorOut.g>255)colorOut.g=255;
  colorOut.b = abs(z)*CMULT+COFF;
  if(colorOut.b<0)colorOut.b=0;
  if(colorOut.b>255)colorOut.b=255;
    
  temp = colorOut.r;
  temp = temp<<8;
  temp = temp + colorOut.g;
  temp = temp<<8;
  temp = temp + colorOut.b;
  
  return(temp);
}

//================================================================================
// This function takes floating point values from the accelerometer
// and returns a floating point hue value on the 0 to 360 degree range
unsigned long getHue(float x, float y, float z){

  #define CMULT (float)120;
  #define COFF 10;
  unsigned long temp;
  
  colorOut.r = abs(x)*CMULT+COFF;
  if(colorOut.r<0)colorOut.r=0;
  if(colorOut.r>255)colorOut.r=255;
  colorOut.g = abs(y)*CMULT+COFF;
  if(colorOut.g<0)colorOut.g=0;
  if(colorOut.g>255)colorOut.g=255;
  colorOut.b = abs(z)*CMULT+COFF;
  if(colorOut.b<0)colorOut.b=0;
  if(colorOut.b>255)colorOut.b=255;

  // retain H, replace S and V
  colorIn = rgb2hsv(colorOut);

  return(colorIn.h);
}

//================================================================================

// Beat detector 

// 20 - 200hz Single Pole Bandpass IIR Filter
float bassFilter(float sample) {
    static float xv[3] = {0,0,0}, yv[3] = {0,0,0};
    xv[0] = xv[1]; xv[1] = xv[2]; 
//    xv[2] = (sample) / 3.f; // change here to values close to 2, to adapt for stronger or weeker sources of line level audio  
    // TEST
    // xv[2] = (sample) / 1.0f; // change here to values close to 2, to adapt for stronger or weeker sources of line level audio  
    xv[2] = (sample * 0.5);
    

    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] = (xv[2] - xv[0])
        + (-0.7960060012f * yv[0]) + (1.7903124146f * yv[1]);
    return yv[2];
}

// 10hz Single Pole Lowpass IIR Filter
float envelopeFilter(float sample) { //10hz low pass
    static float xv[2] = {0,0}, yv[2] = {0,0};
    xv[0] = xv[1]; 
//    xv[1] = sample / 50.f;
    xv[1] = sample / 250.f;
    yv[0] = yv[1]; 
//    yv[1] = (xv[0] + xv[1]) + (0.9875119299f * yv[0]);
    yv[1] = (xv[0] + xv[1]) + (0.98f * yv[0]);
    return yv[1];
}

// 1.7 - 3.0hz Single Pole Bandpass IIR Filter
float beatFilter(float sample) {
    static float xv[3] = {0,0,0}, yv[3] = {0,0,0};
    xv[0] = xv[1]; xv[1] = xv[2]; 
    xv[2] = sample / 2.7f;
    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] = (xv[2] - xv[0])
        + (-0.7169861741f * yv[0]) + (1.4453653501f * yv[1]);
    return yv[2];
}


//=================================================================================


//=================================================================================

void setup() {

  // Pin 10 is the SPISEL line, set it HIGH to use the LEDs or the Piezo, both are enabled
  // thru the 3.3 to 5 volt level shifter.  Pin 10 LOW to use spi with the accelerometer.
  pinMode(10,OUTPUT);
  digitalWrite(10,LOW);

  // Startup tone
  tone(8,440,TONETIME);
  delay(TONETIME);
  tone(8,880,TONETIME);
  delay(TONETIME);
  tone(8,1760,TONETIME);
  delay(TONETIME);
  
//  delay(TONETIME);
//  tone(8,110,TONETIME);
//  delay(TONETIME);

  // LED strip
  digitalWrite(10,HIGH);
  strip.begin(); // Initialize pins for output
  strip.clear();
  strip.show();  // Turn all LEDs off ASAP
  digitalWrite(10,LOW);
   
  // Xbee 
  Serial.begin(57600);   // Serial comms with the Xbee

  // Accelerometer
  a1.settings.adcEnabled = 0;
  a1.settings.tempEnabled = 0;
  a1.settings.accelSampleRate = 25;  // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  a1.settings.accelRange = 4;        // Max G force readable.  Can be: 2, 4, 8, 16
  a1.settings.xAccelEnabled = 1;
  a1.settings.yAccelEnabled = 1;
  a1.settings.zAccelEnabled = 1;
  a1.begin();
  
  // Beat detector on mic
  sbi(ADCSRA,ADPS2);  // Set ADC to 77khz, max for 10bit
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  pinMode(9, OUTPUT);  // The pin with the LED


}

//=================================================================================

void loop() {

  // Read the 3d accelerometer on the SPI
  digitalWrite(4,HIGH);
  ax = a1.readFloatAccelX();
  ay = a1.readFloatAccelY();
  az = a1.readFloatAccelZ();
  digitalWrite(4,LOW);

  // Start of beat detect
  aread = analogRead(0)-512;
  value = (float)aread;
  if(value<0)value=-value;    // absolute value
  if(value<10) value = 0;    // also zero low volume
  envelope = envelopeFilter(value);   // IIR lowpass filter

  // Set brightness based on beat envelope
  bright = envelope*0.01;
  if(bright<envelope) bright +=0.00001;
  if(bright>envelope) bright -=0.00001;
  if(bright<0.005) bright=0.005;
  if(bright>1) bright=1.0;  

  // use the accel to chose a base Hue
  hueBase = getHue(ax,ay,az);
  
  findex += finc;
  if(findex>360) findex -= 360;
  
  index = hueBase + findex;
  if(index>360) index-=360;  

  /*
  // increment thru the modes
  // start in mode2, then go to mode3
  if(modei++>2000){
    modei=0;
    mode++;
    if(mode>3)mode=3;
  }
  */

  strip.clear();
  
  // mode 1 is just the 2 last leds
  if(mode==1){
    index2 = index + 0;
    if(index2>360)index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(51,color1);
    
    index2 = index + 60;
    if(index2>360)index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(52,color1);
  }

  // mode 2 is just the inward pointing leds
  if(mode==2){
    strip.setPixelColor(0,xred,xgreen,xblue);
    strip.setPixelColor(16,xred,xgreen,xblue);
    strip.setPixelColor(17,xred,xgreen,xblue);
    strip.setPixelColor(33,xred,xgreen,xblue);
    strip.setPixelColor(34,xred,xgreen,xblue);
    strip.setPixelColor(50,xred,xgreen,xblue);
  }
  
  // mode 3 is the 3 main hoops
  if(mode==3){
    // ABC row 1
    led=0;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(1,color1);
    strip.setPixelColor(15,color1);
    strip.setPixelColor(18,color1);
    strip.setPixelColor(32,color1);
    strip.setPixelColor(35,color1);
    strip.setPixelColor(49,color1);
    
    // ABC row 2
    led=1;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(2,color1);
    strip.setPixelColor(14,color1);
    strip.setPixelColor(19,color1);
    strip.setPixelColor(31,color1);
    strip.setPixelColor(36,color1);
    strip.setPixelColor(48,color1);
    
    // ABC row 3
    led=2;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(3,color1);
    strip.setPixelColor(13,color1);
    strip.setPixelColor(20,color1);
    strip.setPixelColor(30,color1);
    strip.setPixelColor(37,color1);
    strip.setPixelColor(47,color1);
    
    // ABC row 4
    led=3;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(4,color1);
    strip.setPixelColor(12,color1);
    strip.setPixelColor(21,color1);
    strip.setPixelColor(29,color1);
    strip.setPixelColor(38,color1);
    strip.setPixelColor(46,color1);
    
    // ABC row 5
    led=4;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(5,color1);
    strip.setPixelColor(11,color1);
    strip.setPixelColor(22,color1);
    strip.setPixelColor(28,color1);
    strip.setPixelColor(39,color1);
    strip.setPixelColor(45,color1);
    
    // ABC row 6
    led=5;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(6,color1);
    strip.setPixelColor(10,color1);
    strip.setPixelColor(23,color1);
    strip.setPixelColor(27,color1);
    strip.setPixelColor(40,color1);
    strip.setPixelColor(44,color1);
    
    // ABC row 7
    led=6;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(7,color1);
    strip.setPixelColor(9,color1);
    strip.setPixelColor(24,color1);
    strip.setPixelColor(26,color1);
    strip.setPixelColor(41,color1);
    strip.setPixelColor(43,color1);
    
      // ABC top
    led=7;
    index2 = index + led*offset;
    if(index2>360) index2 -= 360;
    color1 = getColor( index2, 1.0, bright);
    strip.setPixelColor(8,color1);
  
  } 

  // Refresh strip
  digitalWrite(10,HIGH);  // need to select strip not accelerometer
  strip.show();                     
  digitalWrite(10,LOW);   // deselect strip

  // Xbee comms
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    
    // mode command
    if(XbeeIn == 1){
      delay(10);
      mode = Serial.read();
    }

    // color command
    if(XbeeIn == 4){      
      delay(10);
      xred = Serial.read();      
      delay(10);
      xgreen = Serial.read();      
      delay(10);
      xblue = Serial.read();      
    }  
  } // end of Xbee comms
  
  delay(DELAY);                        // Variable delay controls apparent speed

}  // end of loop()

//=================================================================================
