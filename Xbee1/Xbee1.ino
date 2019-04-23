// LiquidCrystal - Version: 1.0.7
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);  // Setup the LCD object and assign pins

int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

//=================================================================================

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("Xbee1");

  pinMode(9,INPUT);
  pinMode(A0,INPUT);    // Cant use pins 10,11,12,13 since configured for SPI
  pinMode(A1,INPUT);    // so instead use two of the analog inputs configure for digital
  
  pinMode(A2,OUTPUT);    // Controls backlight, High = on , Low = off
  digitalWrite(A2,HIGH);  // Turn on backlight at startup

  Serial.begin(9600);   // Serial comms with the Xbee
  
}

//=================================================================================


//=================================================================================

void loop() {

  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    Serial.print(XbeeOut);
    XbeeCount++;
    
  
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("Count ");
    lcd.print(XbeeCount);
    lcd.print(" Data ");
    lcd.print(XbeeIn);
    
  }
}

//=================================================================================


