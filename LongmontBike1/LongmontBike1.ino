/* =============================================================
 * Longmont Bike 1  
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
 *  Analog output on pin 9 to a regular LED for beat detection.
 *  2 menu switch inputs on pins 4 and 7.
 *  
 *  LED Layout: 2 sets 0 to 20, and 21 to 41, plus one extra at 42.
 *  
 *  Dr Tom Flint, 27 Jan 2019
 *  Rework for the 3.3 volt ProMini, 22 Feb 2019
 *  Add over-the-air setup, 1 Mar 2019
 *  Adapt for Globe BB-6a, 2 April 2019
 *  From BB7 hack a new Bike Lights version, 13 Nov 2020
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel
#define NUMLEDS 43            // Number of LEDs in strip
#define DELAY 2               // used at end of loop()
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

int ledi = 0;
int xled = 0;
int xred = 0;
int xgreen = 0;
int xblue = 0;

// spirals
int shue = 0;               // spiral base hue
unsigned long scolor[5];    // array of 5 colors
int scount = 0;             // counter for each step
int sstep = 0;              // 0 to 4, index of first fiber
int speriod = 12;           // counts per step, low=fast high=slow
int sbright = 1;            // independent brightness for the spiral
int si = 0;                 // loop counter in spirals
int smax = 120;             // hue limiter
int smin = 25;              // hue limiter
int sindex = smax;
int sincr = -1;           


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
float bright2 = 0.1;

#define NUMMODES 2  
int mode = 0;
int old_mode = -1;

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
  
  // mode 0, sort out the fibers and label groups 
  if(mode==0){
    // TEST force brightness to full while working on fiber placement
    bright=1.0;
    

    // --- Outline of the Boat ------------------
    color1 = getColor(index,1,bright);        // use base color
    strip.setPixelColor(11,color1);           // around spiral to bow
    strip.setPixelColor(32,color1);           // around spiral to rail
    strip.setPixelColor(10,color1);           // stern keel and lamp right
    strip.setPixelColor(31,color1);           // stern keel and lamp left

    // --- Spirals ------------------------------- 
       
    // spiral index (base color) ramps between smin and smax
    sbright = 1;      
/*    
    sindex = sindex + sincr;
    if(sindex>=smax){
      sincr = -1;
    }
    if(sindex<=smin){
      sincr = 1;
    }
    index2=sindex;
*/
    index2=index;
    bright2=sbright;    
    for(si=0;si<5;si++){                      
      scolor[si]=getColor(index2,1,bright2);
      index2=index2+5;                   // offset in hue
      if(index2>360) index2 = index2 - 360;
      bright2 = 0.4 * bright2;                // dim later fibers
    }

    // update the spiral count and step
    scount = scount+1;
    if(scount>speriod){
      scount=0;
      sstep = sstep - 1;
      if(sstep<0) sstep = 4;
    }

    // assign the 5 colors to the 5 fibers based on step
    // use si as a temp index since it should wrap around
    si = sstep;
    strip.setPixelColor(19,scolor[si]);
//    strip.setPixelColor(40,scolor[si]);
    strip.setPixelColor(36,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(18,scolor[si]);
//    strip.setPixelColor(39,scolor[si]);
    strip.setPixelColor(37,scolor[si]);
    
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(17,scolor[si]);
//    strip.setPixelColor(38,scolor[si]);
    strip.setPixelColor(38,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(16,scolor[si]);
//    strip.setPixelColor(37,scolor[si]);
    strip.setPixelColor(39,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(15,scolor[si]);
//    strip.setPixelColor(36,scolor[si]);
    strip.setPixelColor(40,scolor[si]);

    // ----- Water ------------------------- 
    index2 = 240;   // Start with a blue
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(14,color1);
    strip.setPixelColor(35,color1);

    index2 = index2 + 10;
    if(index2>360) index2 = index2 - 360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(13,color1);
    strip.setPixelColor(34,color1);

    index2 = index2 + 10;
    if(index2>360) index2 = index2 - 360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(12,color1);
    strip.setPixelColor(33,color1);

    // --- Fenders --------------------------
    index2 = 0;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(24,color1);         // front headlight loop
    strip.setPixelColor(3,color1);          // front teardrop
    strip.setPixelColor(42,color1);         // rear teardrop

    // --- Tail Lights --------------------
    index2 = 120;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(1,color1);          // low numbers are right side
    strip.setPixelColor(22,color1);

    // --- Basket -------------------------
    index2 = 105;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(0,color1);          // loop to tail
    strip.setPixelColor(21,color1);         // loop to spotlight

    // --- Halo ------------------------------
    index2 = 60;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(20,color1);

    // --- Wand -------------------------------
    index2 = 180;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(41,color1);

}
  
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

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

    // select command
    if(XbeeIn == 3){      
      delay(10);
      xled = Serial.read();   
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
    
  }

  
//  delay(DELAY);                        // Variable delay controls apparent speed

}

//=================================================================================
