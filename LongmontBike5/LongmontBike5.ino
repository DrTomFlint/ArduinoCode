/* =============================================================
 * Longmont Bike 5
 * Make some updates for "Hedwigs Holiday"  10 Dec 2021.
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
 *  Updates for Hedwigs Holiday, 10 Dec 2021
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel
#define NUMLEDS 209            // Number of LEDs in strip
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
int speriod = 24;           // counts per step, low=fast high=slow
int speriod_cmd = 24;       // spiral period command, prior rate limit
float sbright = 0;            // independent brightness for the spiral
float sbright_cmd=0;          // spiral brightness command, prior rate limit
int si = 0;                 // loop counter in spirals
int smax = 120;             // hue limiter
int smin = 25;              // hue limiter
int sindex = smax;
int sincr = -1;           

// Furry String
int furled1 = 0;
int furled2 = 7;
int furled3 = 30;
int furcount = 0;
int furperiod = 3;
unsigned long furcolor1 = 0L;
unsigned long furcolor2 = 0L;
unsigned long furcolor3 = 0L;

// WanderLeds
#define wMax 10        // max number sprites
int wNum = 0;         // number of active sprites
int wi = 0;           // index
float wSpeedMin=-0.4;   
float wSpeedMax=0.4;
float wSpeedInc=0.01;
float wLocationMax=165;
//float wLocation[wMax] = {48,48,48,48,48,48,48,48,48,48};
float wLocation[wMax] = {0,0,0,0,0,0,0,0,0,0};
float wSpeed[wMax]= {0.1, 0.2, 0.3, 0.2, 0.2, -0.1, -0.2, -0.3, -0.2 -0.2};
float wHue[wMax]= {0, 10, 20, -10, -20, 15, 25, -15, -25};
float wBright[wMax]= {1,1,1,1,1,1,1,1,1,1};
float wDim[wMax] = {0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01};
int wSleep[wMax]= {0,0,40,50,60,70,80,90,100,110};
int wSleepMax = 2000;

// Colors
float bright = 0.0;      
float bright_cmd = 0.0; 
float bright2 = 0.0;
float findex = 120;       // Keep track of hue with a float, limit loop delay
float finc = 0.05;      // increment much less than 1.0
float hueBase = 0;
int led = 0;
int index = 0;
int index2 = 0;
float offset = 20;
int increment2 = 1;     // hue offset in rainbow mode

unsigned long color1;   // hue (HSV) 
float s1 = 1.0;         // saturation (HSV)

#define MAXMODE 3  
int mode = 1;           
int old_mode = 2;
#define MAXLEVEL 3  
int level = 3;
int old_level = 2;
#define MAXCOLOR 3  
int color = 0;
int old_color = -1;
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
unsigned long getColorOld(float h, float s, float v){
  
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
// Use the Adafruit DotStar libraries as much as possible

unsigned long getColor(float h, float s, float v){
  
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
  
  // Beat detector on mic
  sbi(ADCSRA,ADPS2);  // Set ADC to 77khz, max for 10bit
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  pinMode(9, OUTPUT);  // The pin with the LED

  // Toggle menu switch on pins 4 and 7
  pinMode(4,INPUT);
  pinMode(7,INPUT);

}

//=================================================================================

void loop() {

  // Read the 3d accelerometer on the SPI
  digitalWrite(4,HIGH);
  ax = a1.readFloatAccelX();
  ay = a1.readFloatAccelY();
  az = a1.readFloatAccelZ();
  digitalWrite(4,LOW);

/*
  // Start of beat detect
  aread = analogRead(0)-512;
  value = (float)aread;
  if(value<0)value=-value;    // absolute value
  if(value<10) value = 0;    // also zero low volume
  envelope = envelopeFilter(value);   // IIR lowpass filter
*/
    
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

    // color command
    if(XbeeIn == 4){      
      delay(10);
      xred = Serial.read();      
      delay(10);
      xgreen = Serial.read();      
      delay(10);
      xblue = Serial.read();      
    }  
    
    // mode set command
    if(XbeeIn == 5){      
      delay(10);
      mode = Serial.read();   
      if(mode<0)mode=0;
      if(mode>MAXMODE)mode=MAXMODE;
    }   

  }


/*
  // Set brightness based on beat envelope
  bright = envelope*0.01;
  if(bright<envelope) bright +=0.00001;
  if(bright>envelope) bright -=0.00001;
  if(bright<0.005) bright=0.005;
  if(bright>1) bright=1.0;  

  // use the accel to chose a base Hue
  // hueBase = getHue(ax,ay,az);
  
  scolor[0]=getColor(120,1,sbright);
  scolor[1]=getColor(105,1,sbright*0.7);
  scolor[2]=getColor(60,1,sbright*0.5);
  scolor[3]=getColor(0,1, sbright*0.4);
  scolor[4]=getColor(250,1,sbright*0.25);
  */

  // Read 4 toggle switch input, normally high, pulled low when toggle is active
  buttons_in = 0;
  if(digitalRead(4)==HIGH) buttons_in +=1;
  if(digitalRead(7)==HIGH) buttons_in +=2;
  if(buttons_in != old_buttons){
    // got a new button press
    old_buttons = buttons_in;

    if(buttons_in == 1){
      mode +=1;
//      if(mode>MAXMODE) mode=0;
      if(mode>MAXMODE) mode=1;
    }
    if(buttons_in == 2){
      level +=1;
//      if(level>MAXLEVEL) level=0;
      if(level>MAXLEVEL) level=1;
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
    if(level==0) {bright_cmd=0; sbright_cmd=0; speriod_cmd=12; furperiod=12; wNum=1; wSpeedMax=0.2, wSpeedMin=-0.2; wSleepMax=2000;}
    if(level==1) {bright_cmd=0.1; sbright_cmd=0.5; speriod_cmd=9; furperiod=9; wNum=2; wSpeedMax=0.2, wSpeedMin=-0.2; wSleepMax=2000;}
    if(level==2) {bright_cmd=0.4; sbright_cmd=0.75; speriod_cmd=3; furperiod=3; wNum=6; wSpeedMax=0.3, wSpeedMin=-0.3; wSleepMax=600;}
    if(level==3) {bright_cmd=1.0; sbright_cmd=1.0; speriod_cmd=1; furperiod=1; wNum=10; wSpeedMax=0.5, wSpeedMin=-0.5; wSleepMax=200;}
  
    if(bright<bright_cmd)bright += 0.001;   // range 0 to 1.0
    if(bright>bright_cmd)bright -=0.001;
    
    if(sbright<sbright_cmd)sbright +=0.001;
    if(sbright>sbright_cmd)sbright -=0.001;
    
    if(speriod<speriod_cmd)speriod++;       // 6 is fast, 24 is slow
    if(speriod>speriod_cmd)speriod--;

    
  }

  // ***************** mode -1, Startup *******************************************************
  if(mode==-1){
    
    color1 = getColor(index,1,bright);        // use base color
    for(led=0;led<NUMLEDS;led++){
      strip.setPixelColor(led,(unsigned long)0);
    }
  }
        
  // ***************** mode 0, Off *******************************************************
  if(mode==0){
    for(led=0;led<NUMLEDS;led++){
      strip.setPixelColor(led,(unsigned long)0);
    }
  }

// ***************** mode 4, WanderLeds *******************************************************

  if(mode==4){

    index2=index;
    bright2=bright;
    
    strip.clear();

    // loop over active sprites
    for(wi=0;wi<wNum;wi++){

      // check if this sprite is sleeping
      if(wSleep[wi]>0){
        wSleep[wi]--;
        if(wSleep[wi]==0){    // NEW SPRITE:
//          wLocation[wi]=random(208);        // random start
          wLocation[wi]=91-43;        // bow start
          wSpeed[wi]=random(20)*0.01;       // randomize the speed
          if(random(100)<50){               // random left/right
            wSpeed[wi]= -wSpeed[wi];
          }
          wHue[wi]=random(60);           // start with random color offset
          wBright[wi]=bright2;
//          wDim[wi]= bright2 / (100 + random(200));          // should dim to zero by end of strip
          wDim[wi]= bright2 / (400 + random(200));          // dim slowly
        }
        
      }else{
      
        // adjust the speed
        if(random(20)<2){
          wSpeed[wi] = wSpeed[wi]+wSpeedInc;
          if(wSpeed[wi]>wSpeedMax) wSpeed[wi]=wSpeedMax;
        }
        if(random(20)<2){
          wSpeed[wi] = wSpeed[wi]-wSpeedInc;
          if(wSpeed[wi]<wSpeedMin) wSpeed[wi]=wSpeedMin;
        }

        // adjust the brightness, when dim then sleep it
        wBright[wi] = wBright[wi] - wDim[wi];
        if(wBright[wi] <= 0) {
          wBright[wi]=0;
          wSleep[wi]=random(wSleepMax);
        }
        
        // adjust the (float) location
        wLocation[wi] += wSpeed[wi];
        if(wLocation[wi]>wLocationMax) wLocation[wi] -= wLocationMax;
        if(wLocation[wi]<0) wLocation[wi] += wLocationMax;
  
        // set the led
        index2=index+wHue[wi];        // base color plus offset
        if(index2>360) index2-=360;
        if(index2<0)  index2+=360;
        color1 = getColor(index2,1,wBright[wi]);   
        led = wLocation[wi]+43;
        strip.setPixelColor(led,color1);      
      }
    }

  }

// ***************** mode 5, BowspritLeds *******************************************************

  if(mode==5){

    index2=index;
    bright2=bright;
    
    strip.clear();

    // TEST
//    color1 = getColor(index,1,bright);        // TEST FIBER
//    strip.setPixelColor(4,color1);            // test fiber

    // --- Outline of the Boat ------------------
    color1 = getColor(index,1,bright);        // use base color
    strip.setPixelColor(11,color1);           // around spiral to bow
    strip.setPixelColor(32,color1);           // around spiral to rail
    strip.setPixelColor(10,color1);           // stern keel and lamp right
    strip.setPixelColor(31,color1);           // stern keel and lamp left

    strip.setPixelColor(91,color1);           // bowmost fur light
    bright2=bright*0.7;
    color1 = getColor(index,1,bright2);        // set next couple leds dimmer
    strip.setPixelColor(90,color1);          
    strip.setPixelColor(92,color1);           
    bright2=bright*0.4;
    color1 = getColor(index,1,bright2);        // set next couple leds dimmer
    strip.setPixelColor(89,color1);          
    strip.setPixelColor(93,color1);           
    bright2=bright*0.2;
    color1 = getColor(index,1,bright2);        // set next couple leds dimmer
    strip.setPixelColor(88,color1);          
    strip.setPixelColor(94,color1);           
    
    strip.setPixelColor(42,color1);         // rear teardrop
    strip.setPixelColor(1,color1);          // tail lights low numbers are right side
    strip.setPixelColor(22,color1);

    
    // loop over all sprites
    for(wi=0;wi<wNum;wi++){

      // check if this sprite is sleeping
      if(wSleep[wi]>0){
        wSleep[wi]--;
        if(wSleep[wi]==0){    // NEW SPRITE:
          wLocation[wi]=0;                 // start at bow
          wSpeed[wi]=random(20)*0.01+0.2;   // randomize the speed, not too slow
          wHue[wi]=index-10+random(20);   // slight variations from base color
          if(wHue[wi]>360) wHue[wi] -= 360; // apply wrap arounds
          if(wHue[wi]<0) wHue[wi] +=360;
          wBright[wi]=bright2;
          wDim[wi]= bright2 / (100 + random(200));          // should dim to zero by end of strip
                   
        }
        
      }else{

        // adjust the brightness, when dim then sleep it
        wBright[wi] = wBright[wi] - wDim[wi];
        if(wBright[wi] < 0) {
          wBright[wi] = 0;
          wSleep[wi]=random(wSleepMax);
        }
                      
        // adjust the (float) location, sleep at end-of-run
        wLocation[wi] += wSpeed[wi];
        if(wLocation[wi]>77){
          wSleep[wi] = random(wSleepMax);
        }
  
        // set the leds
        color1 = getColor(wHue[wi],1,wBright[wi]);   
        led=wLocation[wi];
        strip.setPixelColor(91+led,color1);       // left side is continuous
        if(led<49){                               // right side has a break
          strip.setPixelColor(91-led,color1); 
        }else{
          strip.setPixelColor(257-led,color1); 
        }
            
      }
    }


  }

  // ***************** mode 3, Test *******************************************************
  if(mode==3){

    index2=index;
    bright2=bright;
    color1 = getColor(index2,1,bright2);        // use base color

    strip.clear();
    
    // frontmost led
    strip.setPixelColor(91,color1);

    // side leds
    for(led=1;led<79;led++){
      index2-=increment2;
      if(index2<0) index2+=360;
      color1 = getColor(index2,1,bright2);       
      strip.setPixelColor(91+led,color1);   // left side is continuous
      if(led<49){
        strip.setPixelColor(91-led,color1); 
      }else{
        strip.setPixelColor(257-led,color1); 
      }
    }

    // 10 rear sides
//    color1 = getColor(120,1,bright);        // red
//    for(led=170;led<180;led++){
//      strip.setPixelColor(led,color1);            // center
//    }

  }

  // ***************** mode 1, Rainbow Flow, circulate the hues, slightly vary over the fibers
  if(mode==1){

    strip.clear();
//    color1 = getColor(index,1,bright);        // TEST FIBER
//    strip.setPixelColor(4,color1);            // test fiber
    
    // --- Outline of the Boat ------------------
    color1 = getColor(index,1,bright);        // use base color
    strip.setPixelColor(11,color1);           // around spiral to bow
    strip.setPixelColor(32,color1);           // around spiral to rail
    strip.setPixelColor(10,color1);           // stern keel and lamp right
    strip.setPixelColor(31,color1);           // stern keel and lamp left

    // --- Spirals ------------------------------- 
    index2=index;
    bright2=sbright;    
    // select the 5 colors
    for(si=0;si<5;si++){                      
      scolor[si]=getColor(index2,1,bright2);
      index2=index2+5;                        // offset in hue
      if(index2>360) index2 = index2 - 360;
      bright2 = 0.6 * bright2;                // dim later fibers
    }
    // update the spiral count and step, rate limit changes in period
    if(speriod<speriod_cmd) speriod++;
    if(speriod>speriod_cmd) speriod--;
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
    strip.setPixelColor(36,scolor[si]);
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(18,scolor[si]);
    strip.setPixelColor(37,scolor[si]);
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(17,scolor[si]);
    strip.setPixelColor(38,scolor[si]);
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(16,scolor[si]);
    strip.setPixelColor(39,scolor[si]);
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(15,scolor[si]);
    strip.setPixelColor(40,scolor[si]);

    // start from base hue
    index2=index;

    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    
    // ----- Water ------------------------- 
    strip.setPixelColor(14,color1);
    strip.setPixelColor(35,color1);int startcount = 0;


    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(13,color1);
    strip.setPixelColor(34,color1);

    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(12,color1);
    strip.setPixelColor(33,color1);

    // --- Fenders --------------------------
    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(24,color1);         // front headlight loop
    strip.setPixelColor(3,color1);          // front teardrop
    strip.setPixelColor(42,color1);         // rear teardrop

    // --- Tail Lights --------------------
    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(1,color1);          // low numbers are right side
    strip.setPixelColor(22,color1);

    // --- Basket -------------------------
    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(0,color1);          // loop to tail
    strip.setPixelColor(21,color1);         // loop to spotlight

    // --- Halo ------------------------------
    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(20,color1);

    // --- Wand -------------------------------
    index2+=increment2;
    if(index2>360) index2-=360;
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(41,color1);

    // --- Furry String -------------------------    
    index2=index;
    bright2=bright*0.3;
    color1 = getColor(index2,1,bright2);        // use base color    
    strip.setPixelColor(91,color1);             // frontmost led

    furcount++;
    if(furcount>furperiod){
      furcount=0;
      furled1++;                                // increment and wrap
      if(furled1>78){furled1=0;furcolor1=color1;}    
      furled2++;
      if(furled2>78){furled2=0;furcolor2=color1;}
      furled3++;
      if(furled3>78){furled3=0;furcolor3=color1;}
    }
    
    strip.setPixelColor(91+furled1,furcolor1);       // left side is continuous
    if(furled1<49){                               // right side has a break
      strip.setPixelColor(91-furled1,furcolor1); 
    }else{
      strip.setPixelColor(257-furled1,furcolor1); 
    }
    strip.setPixelColor(91+furled2,furcolor2);       // left side is continuous
    if(furled2<49){                               // right side has a break
      strip.setPixelColor(91-furled2,furcolor2); 
    }else{
      strip.setPixelColor(257-furled2,furcolor2); 
    }
    strip.setPixelColor(91+furled3,furcolor3);       // left side is continuous
    if(furled3<49){                               // right side has a break
      strip.setPixelColor(91-furled3,furcolor3); 
    }else{
      strip.setPixelColor(257-furled3,furcolor3); 
    }

    // --- Furry Back -----------------------------
//    color1 = getColor(120,1,0L);            // 120=red
//    for(led=170;led<180;led++){                 // 10 back leds
//      strip.setPixelColor(led,color1);        
//    }

    
  }

  // ******************  mode 2, Basic Ride, green fenders, red tail lights, spiral runs
  if(mode==2){
    strip.clear();
    // --- Outline of the Boat ------------------
    color1 = getColor(index,1,bright);        // use base color
    strip.setPixelColor(11,color1);           // around spiral to bow
    strip.setPixelColor(32,color1);           // around spiral to rail
    strip.setPixelColor(10,color1);           // stern keel and lamp right
    strip.setPixelColor(31,color1);           // stern keel and lamp left

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
    index2 = 0;   // full green
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
    index2 = 105; // orange
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(0,color1);          // loop to tail
    strip.setPixelColor(21,color1);         // loop to spotlight

    // --- Halo ------------------------------
    index2 = 60;  // yellow
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(20,color1);

    // --- Wand -------------------------------
    index2 = 180;   // purple
    color1 = getColor(index2,1,bright);
    strip.setPixelColor(41,color1);


    // --- Furry String -------------------------    
    index2=index;
    bright2=bright*0.3;
    color1 = getColor(index2,1,bright2);        // use base color    
    strip.setPixelColor(91,color1);             // frontmost led

    furcount++;
    if(furcount>furperiod){
      furcount=0;
      furled1++;                                // increment and wrap
      if(furled1>78){furled1=0;furcolor1=color1;}    
      furled2++;
      if(furled2>78){furled2=0;furcolor2=color1;}
      furled3++;
      if(furled3>78){furled3=0;furcolor3=color1;}
    }
    
    strip.setPixelColor(91+furled1,furcolor1);       // left side is continuous
    if(furled1<49){                               // right side has a break
      strip.setPixelColor(91-furled1,furcolor1); 
    }else{
      strip.setPixelColor(257-furled1,furcolor1); 
    }
    strip.setPixelColor(91+furled2,furcolor2);       // left side is continuous
    if(furled2<49){                               // right side has a break
      strip.setPixelColor(91-furled2,furcolor2); 
    }else{
      strip.setPixelColor(257-furled2,furcolor2); 
    }
    strip.setPixelColor(91+furled3,furcolor3);       // left side is continuous
    if(furled3<49){                               // right side has a break
      strip.setPixelColor(91-furled3,furcolor3); 
    }else{
      strip.setPixelColor(257-furled3,furcolor3); 
    }

    // --- Spirals -------------------------------        
    // first color is base hue
    index2=index;
    bright2=sbright;    
    for(si=0;si<5;si++){                      
      scolor[si]=getColor(index2,1,bright2);
      index2=index2+5;                   // offset in hue
      if(index2>360) index2 = index2 - 360;
      bright2 = 0.4 * bright2;                // dim later fibers
    }

    // update the spiral count and step, rate limit changes in period
    if(speriod<speriod_cmd) speriod++;
    if(speriod>speriod_cmd) speriod--;
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
    strip.setPixelColor(36,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(18,scolor[si]);
    strip.setPixelColor(37,scolor[si]);
    
    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(17,scolor[si]);
    strip.setPixelColor(38,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(16,scolor[si]);
    strip.setPixelColor(39,scolor[si]);

    si = si+1;
    if(si>=5) si=0;        
    strip.setPixelColor(15,scolor[si]);
    strip.setPixelColor(40,scolor[si]);


}   // ************** end of modes ********************************
  
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
