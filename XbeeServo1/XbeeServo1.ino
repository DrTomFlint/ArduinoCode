#include <LiquidCrystal.h>
#include <Servo.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);  // Setup the LCD object and assign pins

int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

Servo s1;

//=================================================================================

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("XbeeServo1");

  pinMode(9,INPUT);
  pinMode(A0,INPUT);    // Cant use pins 10,11,12,13 since configured for SPI
  pinMode(A1,INPUT);    // so instead use two of the analog inputs configure for digital
  
  pinMode(A2,OUTPUT);    // Controls backlight, High = on , Low = off
  digitalWrite(A2,HIGH);  // Turn on backlight at startup

  Serial.begin(9600);   // Serial comms with the Xbee

  s1.attach(8);     // Attach servo #1 to GPIO8  
}

//=================================================================================


//=================================================================================

void loop() {

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
}

//=================================================================================


