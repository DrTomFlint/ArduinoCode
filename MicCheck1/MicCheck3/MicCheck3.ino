/*=================================================================================
Configured with servo, accelerometer on I2C, LCD still using up a lot of pins but
handy for debug.  Xbee interface so must disconnect RX pin to program over USB.

In making the loop real-time, something has gone amiss
might be the I2C bus?  Not quite sure yet


=================================================================================  */

#include <LiquidCrystal.h>
#include <Servo.h>
#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"

// LCD on benchtop
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);  // Setup the LCD object and assign pins

// Xbee comms
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

// Servo
Servo s1;

// Accelerometer
LIS3DH a1( I2C_MODE, 0x18 );
float ax, ay, az;     // accelerometer readings
float am;             // magnitude squared of a(xyz)
float axm, aym, azm;  // magnitude = abs(xyz)
int ana1, ana2, ana3;  // analog innputs 10 bit, ana3 may have thermal data

// Beat detector
#define SAMPLEPERIODUS 200

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//=================================================================================

void setup() {

  // Three buttons for use with LCD menus
  pinMode(9,INPUT);
  pinMode(A3,INPUT);    // Cant use pins 10,11,12,13 since configured for SPI
  pinMode(A1,INPUT);    // so instead use two of the analog inputs configure for digital  
  
  // LCD
  pinMode(A2,OUTPUT);    // Controls backlight, High = on , Low = off
  digitalWrite(A2,HIGH);  // Turn on backlight at startup
  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("MicCheck3");

  // Xbee
  Serial.begin(9600);   // Serial comms with the Xbee

  // Servo
  s1.attach(8);     // Attach servo #1 to GPIO8  

  // Piezo for sounds
  pinMode(12,OUTPUT);
  tone(12,440,2000);

  // Beat detector on mic
  sbi(ADCSRA,ADPS2);  // Set ADC to 77khz, max for 10bit
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  pinMode(10, OUTPUT);  // The pin with the LED

/*
  // Accelerometer
  a1.settings.adcEnabled = 1;
  a1.settings.tempEnabled = 1;
  a1.settings.accelSampleRate = 10;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  a1.settings.accelRange = 4;      //Max G force readable.  Can be: 2, 4, 8, 16
  a1.settings.xAccelEnabled = 1;
  a1.settings.yAccelEnabled = 1;
  a1.settings.zAccelEnabled = 1;
  if(a1.begin() !=0 ){
    lcd.setCursor(0,0);
    lcd.print("Fail a1     ");
  }
*/
 


}
//=================================================================================
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
    xv[1] = sample / 50.f;
    yv[0] = yv[1]; 
    yv[1] = (xv[0] + xv[1]) + (0.9875119299f * yv[0]);
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

void loop() {

/*
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    Serial.print(XbeeOut);
    XbeeCount++;
    
    if(XbeeIn>0){
      if(XbeeIn<180){
        s1.write(XbeeIn);
      }
    }

    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("# ");
    lcd.print(XbeeCount);
    lcd.print(" Data ");
    lcd.print(XbeeIn);
  }

*/    

/*
// Tests for the 3d accelerometer on the I2C port

    ax = a1.readFloatAccelX();
    ay = a1.readFloatAccelY();
    az = a1.readFloatAccelZ();
//    am = ax*ax + ay*ay + az*ax;

    axm = abs(ax);
    aym = abs(ay);
    azm = abs(az);
    
    if((axm>1.0)||(aym>1.0)||(azm>1.0)){
//      tone(12,440*am,50);
      lcd.setCursor(12,0);
      if(axm>aym){
        // ay is not the largest value
        if(axm>azm){
          // ax is largest value
          tone(12,110,50);
          lcd.print("X");
        }else{
          // az is largest value
          tone(12,440,50);
          lcd.print("Z");
        }               
      }else{
        // ax is not the largest value
        if(aym>azm){
          // ay is largest value
          tone(12,220,50);
          lcd.print("Y");
        }else{
          // az is largest value
          tone(12,440,50);
          lcd.print("Z");
        }
      }
    }
    
//    ana1 = a1.read10bitADC1();
//    ana2 = a1.read10bitADC2();
//    ana3 = a1.read10bitADC3();
    
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print(ax);
    lcd.print(" ");
    lcd.print(ay);
    lcd.print(" ");
    lcd.print(az);
*/    
/*
    lcd.print(" y=");
    lcd.print(ay);
    lcd.print(" z=");
    lcd.print(az);
    delay(300);
*/
    unsigned long time = micros(); // Used to track rate
    float sample, value, envelope, beat, thresh;
    unsigned char i;
    int aread;
    int duty;

    for(i = 0;;++i){
        // Read ADC and center so +-512
        
        // TEST try to eliminate saturated values
        aread = analogRead(0)-512;
        if(aread>500) aread=0;
        if(aread<-500) aread=0;
        sample = (float)aread;
        //sample = (float)analogRead(0)-512.0f;

        // Filter only bass component
        value = bassFilter(sample);

        // Take signal amplitude and filter
        if(value < 0)value=-value;
        envelope = envelopeFilter(value);

        // Every 200 samples (25hz) filter the envelope 
        if(i == 200) {
                // Filter out repeating bass sounds 100 - 180bpm
                beat = beatFilter(envelope);

                // Threshold it based on potentiometer on AN1
//                thresh = 0.02f * (float)analogRead(1);
                thresh = 0.02f * (float)(1000.0);

                // If we are above threshold, light up LED
             //   if(beat > thresh){
             //     digitalWrite(A5, HIGH);
             //   }else{
             //     digitalWrite(A5, LOW);
             //   }
                duty = beat*5.0;
                if(duty>255)duty=255;
                analogWrite(10,duty);
                
                // Reset sample counter
                i = 0;
        }
        // Consume excess clock cycles, to keep at 5000 hz
        for(unsigned long up = time+SAMPLEPERIODUS; time > 20 && time < up; time = micros());
    }  
  
}

//=================================================================================
