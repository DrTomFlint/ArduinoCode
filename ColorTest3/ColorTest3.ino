// Adafruit DotStar - Version: 1.0.4
#include <Adafruit_DotStar.h>
// LiquidCrystal - Version: 1.0.7
#include <LiquidCrystal.h>

#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

// 12-28-2018 cut off 18 leds from strip
#define NUMPIXELS 12 // Number of LEDs in strip
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
rgb colorOut;
hsv colorIn;

// New for ColorTest1:
int led = 0;
int base = 0;
float hue[7] = {120, 105, 60, 0, 260, 220, 180};
//float hue[7] = {154, 103, 51, 0, 307, 256, 205};
float hueinc[7];
int huestep = 500;
int hues = 7;
int huehold = 3;
int holdi = 0;
int huei=0;
int stepi=0;
int index = 0;
int index2 = 0;
float offset = 2;

int redx, greenx, bluex;

// older
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


//=================================================================================

unsigned long getColor2(float index, float brightness)
{
  int sector;
  float frac;
  float red,grn,blu;
  unsigned long temp;
  
  sector = (int)(index/60);
  frac = (index - sector*60) / 60;
  
  if(sector<0) sector = 0;
  if(sector>5) sector = 5;
  
  switch(sector){
    case 0:
      red = 1;
      grn = frac;
      blu = 0;
      break;
    case 1:
      red = 1-frac;
      grn = 1;
      blu = 0;
      break;
    case 2:
      red = 0;
      grn = 1;
      blu = frac;
      break;
    case 3:
      red = 0;
      grn = 1-frac;
      blu = 1;
      break;
    case 4:
      red = frac;
      grn = 0;
      blu = 1;
      break;
    case 5:
      red = 1;
      grn = 0;
      blu = 1-frac;
      break;
    default:
      red = 0.6;
      grn = 0.6;
      blu = 0.6;
  }
  
  red = red * brightness;
  grn = grn * brightness;
  blu = blu * brightness;
  
  temp = grn;
  temp = temp<<8;
  temp = temp + red;
  temp = temp<<8;
  temp = temp + blu;
  
  return(temp);
  
}

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

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  lcd.begin(16,2);    // is that 16 characters and 2 lines?
  lcd.print("ColorTest3");

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
 
 hueinc[0] = ( hue[1] - hue[0] ) / huestep;
 hueinc[1] = ( hue[2] - hue[1] ) / huestep;
 hueinc[2] = ( hue[3] - hue[2] ) / huestep;
 hueinc[3] = ( hue[4] - 360) / huestep;
 hueinc[4] = ( hue[5] - hue[4] ) / huestep;
 hueinc[5] = ( hue[6] - hue[5] ) / huestep;
 hueinc[6] = ( hue[0] - hue[6] ) / huestep;
 
}

//=================================================================================

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
    // hijacked to establish the base color
    if(menui == 2){
      if(base>0) base--;
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
      if(base < 360) base++;
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
    lcd.print(" Base        ");
    lcd.print(base);
  }
  
  
}

//=================================================================================

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
  if(digitalRead(A0)==HIGH) buttons_in +=4;
  if(digitalRead(A1)==HIGH) buttons_in +=8;
  
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

  index += 1;                 // increment the color by 1
  if(index>160) index++;          // if color > ~160(blue) then increment again for faster cycle.
  if(index>=360) index -= 360;
  
  for(led=0;led<12;led++){
   index2 = index + (offset + index/90)*led;
    while(index2>=360) index2 -= 360;
    while(index2<0) index2 += 360;
    
//    color1 = getColor( index2, s1, bright );
    color1 = getColor2( index2, 255*bright );
    strip.setPixelColor(led,color1);
  }

  strip.show();                     // Refresh strip
  delay(speed);                        // Variable delay controls apparent speed

/*
  // Get a look at the RGB numbers
  color1 = getColor2(index, 255*bright);
  
  colorIn.h = index;
  colorIn.s = s1;
  colorIn.v = bright;
  colorOut = hsv2rgb(colorIn);
  colorOut.r *= 255;
  colorOut.g *= 255;
  colorOut.b *= 255;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("i=");
  lcd.print(index);
  lcd.print("    r=");
  lcd.print(colorOut.r);
  lcd.setCursor(0,1);
  lcd.print("g=");
  lcd.print(colorOut.g);
  lcd.print("    b=");
  lcd.print(colorOut.b);
//  lcd.print(" ");
//  lcd.print(greenx);
//  lcd.print(" ");
//  lcd.print(bluex);
*/

}

//=================================================================================


