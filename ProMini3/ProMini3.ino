/* =============================================================
 *  ProMini3
 *  
 *  Test rig on the benchtop
 *  
 *  LED string on SPI port pins 11 and 13
 *  Xbee on the UART pins RXI TXD
 *  Piezo for tone on pin 2
 *  Accelerometer on I2C bus pins SDA SCL
 *  
 *  Dr Tom Flint, 27 Jan 2019
 *  Rework for the 3.3 volt ProMini, 22 Feb 2019
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#include "SparkFunLIS3DH.h"   // 3d accelerometer
#include "Wire.h"             // i2c for accel

#define NUMPIXELS 30 // Number of LEDs in strip
#define DELAY 150    // used at end of loop()
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
int index = 0;
int index2 = 0;
float offset = 60;
unsigned long color1;   // hue (HSV) 
float s1 = 1.0;         // saturation (HSV)
float bright = 0.2;     // value (HSV)

int redx, greenx, bluex;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;


// Accelerometer
LIS3DH a1( I2C_MODE, 0x19 );  // Use I2C and default sparkfun breakout address = 0x19
float ax, ay, az;     // accelerometer readings
float am;             // magnitude squared of a(xyz)
float axm, aym, azm;  // magnitude = abs(xyz)
int ana1, ana2, ana3;  // analog innputs 10 bit, ana3 may have thermal data
int axi, ayi, azi;


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

  // LED strip
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
   
  // Xbee 
//  Serial.begin(19200);   // Serial comms with the Xbee

  // Debugging
  Serial.begin(19200);   // Serial comms to the IDE for debug
  Serial.print("ProMini3: startup Ok");
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  
  // Accelerometer
  a1.settings.adcEnabled = 0;
  a1.settings.tempEnabled = 0;
  a1.settings.accelSampleRate = 25;  // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  a1.settings.accelRange = 4;        // Max G force readable.  Can be: 2, 4, 8, 16
  a1.settings.xAccelEnabled = 1;
  a1.settings.yAccelEnabled = 1;
  a1.settings.zAccelEnabled = 1;
  a1.begin();
  
  Serial.print("ProMini3: accel begin Ok");

}

//=================================================================================

void loop() {

  digitalWrite(3,HIGH);
  
  index++;
  if(index>=360) index -= 360;
  if(index<0) index += 360;
  
  for(led=0;led<NUMPIXELS;led++){
   index2 = index + offset*led;
    while(index2>=360) index2 -= 360;
    while(index2<0) index2 += 360;
    
    color1 = getColor( index2, s1, bright );
    strip.setPixelColor(led,color1);
  }

  strip.show();                     // Refresh strip

  digitalWrite(3,LOW);


/*
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    
    Serial.print(XbeeOut);

    if(XbeeIn == 1){
      tone(2,440,TONETIME);
      delay(TONETIME);
    }
    if(XbeeIn == 2){
      tone(2,880,TONETIME);
      delay(TONETIME);
    }
    if(XbeeIn == 3){
      tone(2,1760,TONETIME);
      delay(TONETIME);
    }
  }
  
*/

  digitalWrite(4,HIGH);

  // Tests for the 3d accelerometer on the I2C port
  ax = a1.readFloatAccelX();
  ay = a1.readFloatAccelY();
  az = a1.readFloatAccelZ();

  digitalWrite(4,LOW);

  Serial.print("x = ");
  Serial.print(ax);
  Serial.print(" y = ");
  Serial.print(ay);
  Serial.print(" z = ");
  Serial.println(az);
   
  delay(DELAY);                        // Variable delay controls apparent speed

}

//=================================================================================
