/*=================================================================================
Configured with servo, accelerometer on I2C, LCD still using up a lot of pins but
handy for debug.  Xbee interface so must disconnect RX pin to program over USB.

 THIS IS BROKEN, MIGHT BE HARDWARE, FIRST CALL INTO LIS3DH FREEZES EXECUTION

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


//=================================================================================

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("Accel1");

  // buttons for lcd menus
  pinMode(9,INPUT);
  pinMode(A3,INPUT);    // Cant use pins 10,11,12,13 since configured for SPI
  pinMode(A1,INPUT);    // so instead use two of the analog inputs configure for digital

  // lcd backlight
  pinMode(A2,OUTPUT);    // Controls backlight, High = on , Low = off
  digitalWrite(A2,HIGH);  // Turn on backlight at startup

  // Xbee
  Serial.begin(9600);   // Serial comms with the Xbee

  // servo
  s1.attach(8);     // Attach servo #1 to GPIO8  

  // Piezo
  pinMode(12,OUTPUT);
  tone(12,440,1000);

  // Beat detector on mic
  pinMode(A5, OUTPUT);  //The pin with the LED
  digitalWrite(A5,LOW);

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

  lcd.print(" OK");
}

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

    delay(300);
    
/*
    lcd.print(" y=");
    lcd.print(ay);
    lcd.print(" z=");
    lcd.print(az);
    delay(300);
*/
  
}

//=================================================================================
