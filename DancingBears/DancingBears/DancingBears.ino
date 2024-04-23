/* =============================================================
 * DancingBears
 * Base on Longmont Strip1
 * Runs on 5V Arduino with 16 MHz clock (3.3v uses 8 MHz clock), 
 * Xbee #2
 *    
 * LED string on SPI port pins 11=MOSI and 13=CLK.
 * Xbee on the UART pins RXI TXD, 57600 baud to match bootloader, 
 * DIO3 is tied to GRN (reset thru 0.1 uF cap), "digital line passing" 
 * forwards DIO3 of master, which is tied to DTR so the Arduino IDE 
 * can reset chip and start programming sequence.  
 *  
 * 2 menu switch inputs on pins 5 and 7, active low
 *  
 * LED Layout: 85 Leds, only half are on bears, other half are between
 *  
 *  
 ===============================================================*/
 
#include <Adafruit_DotStar.h>
#include <SPI.h>              // spi for LED string
#define NUMLEDS 85            // Number of LEDs in strip

Adafruit_DotStar strip = Adafruit_DotStar(NUMLEDS, DOTSTAR_BRG);

// Colors
unsigned long color0;   // hue (HSV) 
float bright = 0.0;      
float bright_cmd = 0.0; 
float bright2 = 0.0;

float findex = 120;       // Keep track of hue with a float, limit loop delay
float finc = 3.0;      // increment much less than 1.0
int led = 0;
float index = 0;
float index2 = 0;

// location of lit bears
int bear[5] = {0,5,10,15,20};
int i=0;  // index
float bearBase = 0;
float bearInc = 0.1;

// hue for each bear
unsigned long bhue[5] = {0,72,2*72,3*72,4*72};

//#define MAXMODE 10  
#define MAXMODE 1  
int mode = 0 ;           
int old_mode = 1;

#define MAXLEVEL 9  
int level = 3;
int old_level = 0;

int buttons_in = 3;
int old_buttons = 3;

// Xbee
int XbeeIn = 0;
int XbeeOut = 0;
int XbeeCount = 0;

//=================================================================================
// This function takes floating point values for 
// hue = h range 0 to 360
// saturation = s range 0 to 1.0
// value = v range 0 to 1.0
// and returns a 32 bit color code for the RGB values 
// needed for the LED string: 0x00GGRRBB
// Use the Adafruit DotStar libraries as much as possible

unsigned long getColor(float h, float s, float v){
  
  unsigned long colorRaw;
  unsigned long colorFix;
  uint16_t hue;
  uint8_t sat;
  uint8_t val;

  hue = floor(h*65535.0/360.0);
  sat = floor(s*255);
  val = floor(v*255);
  
  colorRaw= strip.ColorHSV(hue,sat,val);
  colorFix = strip.gamma32(colorRaw);

  return(colorFix);
}

//=================================================================================

void setup() {

  // LED strip
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
   
  // Xbee 
//  Serial.begin(19200);   // Serial comms with the Xbee
  Serial.begin(57600);   // Serial comms with the Xbee

  // Debugging
//  Serial.begin(19200);   // Serial comms to the IDE for debug
//  Serial.print("ProMini3: startup Ok");
  pinMode(7,OUTPUT);    // digital flag for timing on scope
  digitalWrite(7,LOW);
    
  // Toggle menu switch on pins 5 and 7
  pinMode(5,INPUT);
  pinMode(7,INPUT);
  
}

//=================================================================================

void loop() {

  // Xbee comms
  if(Serial.available()>0){
    XbeeIn = Serial.read();
    XbeeOut = XbeeIn + 1;
    
    //Serial.print(XbeeOut);

    // if(XbeeIn == 1){
    //   tone(8,440,TONETIME);
    //   delay(TONETIME);
    // }

  }


  // Read 4 toggle switch input, normally high, pulled low when toggle is active
  buttons_in = 0;
  if(digitalRead(5)==HIGH) buttons_in +=1;
  if(digitalRead(7)==HIGH) buttons_in +=2;
  if(buttons_in != old_buttons){
    // got a new button press
    old_buttons = buttons_in;

    if(buttons_in == 1){
      mode +=1;
      if(mode>MAXMODE) mode=0;
    }
    if(buttons_in == 2){
      level +=1;
      if(level>MAXLEVEL) level=0;
    }
  }


  // ******** mode ALL *******************************************************    
  if(mode >= 0){
    // update the hue index, first as float then as an int
    findex += finc;
    if(findex>360) findex -= 360;
    if(findex<0) findex += 360;
    index = findex;
    
    // level will control brightness and speed of the bears
    if(level==0) {bright_cmd=0; finc=0; bearInc = 0;}
    if(level==1) {bright_cmd=0.075; finc=0.1; bearInc = 0.1;}
    if(level==2) {bright_cmd=0.100; finc=0.2; bearInc = 0.1;}
    if(level==3) {bright_cmd=0.125; finc=0.3; bearInc = 0.1;}
    if(level==4) {bright_cmd=0.15; finc=0.4; bearInc = 0.2;}
    if(level==5) {bright_cmd=0.2; finc=0.5; bearInc = 0.3;}
    if(level==6) {bright_cmd=0.3; finc=0.6; bearInc = 0.4;}
    if(level==7) {bright_cmd=0.5; finc=0.7; bearInc = 0.5;}
    if(level==8) {bright_cmd=0.7; finc=1.0; bearInc = 0.6;}
    if(level==9) {bright_cmd=1.0; finc=3.0; bearInc = 0.6;}

// Might ramp brightness  
//    if(bright<bright_cmd)bright += 0.0005;   // range 0 to 1.0
//    if(bright>bright_cmd)bright -=0.0005;
    bright=bright_cmd;      // TEST don't ramp      
  }

  // ***************** mode 0, bears *******************************************************
  if(mode==0){
    
    bearBase += bearInc;
    if(bearBase>10){
      // time to move the bears
      bearBase=0;
      strip.clear();
      for (i=0;i<5;i++){
        bear[i]++;
        if(bear[i]>84) bear[i]=0;
        color0=getColor(bhue[i],1,bright);
        strip.setPixelColor(bear[i],color0);
      }
    }
  }
  
  // ***************** mode 1, flow *******************************************************
  if(mode==1){
    
    for(led=0;led<NUMLEDS;led++){
      index2=index-led*1.5;
      while(index2>360)index2-=360;
      while(index2<0)index2+=360;
      color0 = getColor(index2,1,bright);   
      strip.setPixelColor(led,color0);
    }
  }
        
  
  // ***************** mode 2, all RED for night light *******************************************************
  if(mode==2){
    
    for(led=0;led<NUMLEDS;led++){
      index2=index-led*1.5;
      while(index2>360)index2-=360;
      while(index2<0)index2+=360;
      color0 = getColor(120,1,bright);        // use base color
      strip.setPixelColor(led,color0);
    }

}
  // ************** end of modes ********************************
  
  // Update the strip
  digitalWrite(10,HIGH);    // SPI select for LEDs  
  strip.show();             // Refresh strip
  digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
