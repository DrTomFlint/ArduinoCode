// Adafruit DotStar - Version: 1.0.4
#include <Adafruit_DotStar.h>
// LiquidCrystal - Version: 1.0.7
#include <LiquidCrystal.h>

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 269 // for 30 LED / M older string
//#define NUMPIXELS 144  // for the high density string

#define LENGTH 4      // Number of LED in second chase
#define LENGTH2 12      // Number of LED in lead chase
#define DELAY 20      // Delay slows down the traveling
//#define DEBUG 1

// My strip is Grn Red Blu order of bits.

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);  // Setup the LCD object and assign pins
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

rgb colorOut;
hsv colorIn;

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

float h1index = 0;
float h1step = 0.05;
float s1 = 1.0;
float v1 = 0.1;
unsigned long color1 = 0;
int head  = 0, tail = -LENGTH; // Index of first 'on' and 'off' pixels

float h2index = 60;
float h2step = 1.0;
float s2 = 1.0;
float v2 = 0.1;
unsigned long color2 = 0;
int head2  = 0, tail2 = - LENGTH2; // Index of first 'on' and 'off' pixels

// Menu items
#define NMENU 3
int menui = 0;

#define NBRIGHT 6
float brightTab[NBRIGHT]={0.01, 0.05, 0.1, 0.2, 0.5, 1.0};
int brighti = 2;
float bright;

#define NSPEED 5
float speedTab[NSPEED]={50, 25, 10, 5, 2};
int speedi = 2;
float speed;

#define NSHIFT 5
float shiftTab[NSHIFT]={0.1, 0.2, 0.5, 1.0, 2.0};
int shifti = 2;
float shift;

int buttons_in = 0;
int buttons_out = 0;
int old_buttons = 88;

int menuOn = 1;
#define MENU_MILLIS 4000
unsigned long button_last = 0;
unsigned long button_delta = 0;

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("Pretty Lights 3");

//  pinMode(8,INPUT);
  pinMode(9,INPUT);
  pinMode(A0,INPUT);    // Cant use pins 10,11,12,13 since configured for SPI
  pinMode(A1,INPUT);    // so instead use two of the analog inputs configure for digital
  
  pinMode(A2,OUTPUT);    // Controls backlight, High = on , Low = off
  digitalWrite(A2,HIGH);  // Turn on backlight at startup

 head2 = head2 + NUMPIXELS/4;
 tail2 = tail2 + NUMPIXELS/4;
 
 bright = brightTab[brighti];
 speed = speedTab[speedi];
 shift = shiftTab[shifti];

}

void updateMenu(int button){
  
  // menu down button
  if(button == 1){
    menui--;
    if(menui<0) menui = NMENU -1;
  }

  // menu up button
  if(button == 2){
    menui++;
    if(menui>=NMENU) menui = 0;
  }
  
  // value down button
  if(button == 4){
    
    // brightness menu
    if(menui == 0){
      if(brighti>0) brighti--;
      bright = brightTab[brighti];
      
    }
    
    // speed menu
    if(menui == 1){
      if(speedi>0) speedi--;
      speed = speedTab[speedi];
    }
    
    // shift menu
    if(menui == 2){
      if(shifti>0) shifti--;
      shift = shiftTab[shifti];
    }
    
  }
  
  // value up button
  if(button == 8){
    
    // brightness menu
    if(menui == 0){
      if(brighti < NBRIGHT-1) brighti++;
      bright = brightTab[brighti];
    }
    
    // speed menu
    if(menui == 1){
      if(speedi < NSPEED-1) speedi++;
      speed = speedTab[speedi];
    }

    // shift menu
    if(menui == 2){
      if(shifti < NSHIFT-1) shifti++;
      shift = shiftTab[shifti];
    }
    
  }
  
  // Now that changes are made, update the display
  lcd.setCursor(0,1);
  
  if(menui == 0 ){
    lcd.print(" Brightness  ");
    lcd.print(brighti+1);
  }
  
  if(menui == 1){
    lcd.print(" Speed       ");
    lcd.print(speedi+1);
  }
  
  if(menui == 2){
    lcd.print(" Shift       ");
    lcd.print(shifti+1);
  }
  
  
}

void loop() {
  
  // If backlight time has expired, turn of LCD backlight
  if(  (millis() - button_last) > MENU_MILLIS){
    menuOn = 0;
    digitalWrite(A2,LOW);
    lcd.noDisplay();
  }
  
  // Read 4 button inputs
  buttons_in = 0;
  //if(digitalRead(8)==HIGH) buttons_in +=1;
  if(digitalRead(9)==HIGH) buttons_in +=2;
  if(digitalRead(A1)==HIGH) buttons_in +=4;
  if(digitalRead(A3)==HIGH) buttons_in +=8;
  
  if(buttons_in != old_buttons){
    // got a new button press
    old_buttons = buttons_in;
    button_last = millis();
    if(menuOn ==1){
      // if backlight is already on, update the menu based on button press
      updateMenu(buttons_in);
    }else{
      // if backlight is off, turn it on, reset the backlight timeout, 
      // but do not process the button press
      menuOn = 1;
      digitalWrite(A2,HIGH);
      lcd.display();
    }
  }
  
  // increment the floating point Hue index, range is 0 to 360
  h1index += shift;
  if(h1index>360) h1index -= 360;
  if(h1index<0) h1index += 360;
  color1 = getColor(h1index,s1,bright);

  h2index += shift;
  if(h2index>360) h2index -= 360;
  if(h2index<0) h2index += 360;
  color2 = getColor(h2index,s2,bright);
  
  strip.setPixelColor(head, color1); // 'On' pixel at head
  strip.setPixelColor(tail, 0);     // 'Off' pixel at tail
  strip.setPixelColor(head2, color2); // 'On' pixel at head
  strip.setPixelColor(tail2, 0);     // 'Off' pixel at tail

  strip.show();                     // Refresh strip
  delay(speed);                        // Variable delay controls apparent speed

  if(++head >= NUMPIXELS) head = 0;
  if(++head2 >= NUMPIXELS) head2 = 0;
  if(++tail >= NUMPIXELS) tail = 0; // Increment, reset tail index
  if(++tail2 >= NUMPIXELS) tail2 = 0; // Increment, reset tail index
  
}
