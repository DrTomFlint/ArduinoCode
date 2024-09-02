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
unsigned long color1;   // hue (HSV) 
float bright = 0.0;      
float bright_cmd = 0.0; 
float bright2 = 0.0;

float findex = 120;       // Keep track of hue with a float, limit loop delay
float finc = 3.0;      // increment much less than 1.0
int led = 0;
float index = 0;
float index2 = 0;

// location of lit bears, 3 to 79 odd numbers only
#define MAXBEARS 5
//int bear[MAXBEARS] = {1,6,11,16,21};
int bear[MAXBEARS] = {1,6+10,11+20,16+30,21+40};
int i=0;  // index
float bearCounter = 0;   // 
float bearInc = 0.1;  // speed of animation
int bearNum = 1;      // number of bears to light
int bearSpace = 4;    // space between bears, even numbers only

// location of traces
int trace[MAXBEARS] = {1,6+10,11+20,16+30,21+40};
float traceBright[MAXBEARS] = {1,1,1,1,1};
int j=0;
float traceCounter = 0;
float traceCounter2 = 0;
float traceInc = 0.2;


// hue for each bear, red yellow, green, blue, purple
unsigned long bhue[5] = {120,80,0,240,180};

#define MAXMODE 5  
int mode = 1 ;           
int old_mode = 9;
int delay_mode = 0;

#define MAXLEVEL 2  
int level = 1;
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
//
// XBEE LIST:
// #2  00 13 A2 00 - 41 80 B0 28 Dancing Bears
//
// DrTomFlint 2 Sept 2024
//=================================================================================

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
  old_mode = mode;
  old_level = level;

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
    findex -= finc;
    if(findex>360) findex -= 360;
    if(findex<0) findex += 360;
    index = findex;
    
    // level will control brightness and speed of the bears
    if(level==0) {bright_cmd=0.3; finc=0.1; bearInc = 0.05; bearNum = 3;}
    if(level==1) {bright_cmd=0.5; finc=0.1; bearInc = 0.075; bearNum = 4;}
    if(level==2) {bright_cmd=1.0; finc=0.1; bearInc = 0.1; bearNum = 5;}

// Might ramp brightness  
//    if(bright<bright_cmd)bright += 0.0005;   // range 0 to 1.0
//    if(bright>bright_cmd)bright -=0.0005;
    bright=bright_cmd;      // TEST don't ramp      
  }

  // ***************** mode 0, bears *******************************************************
  if(mode==0){
    
    if(old_mode!=0)  {
      bear[0] = 9+30;
      bear[1] = 17+30;
      bear[2] = 25+30;
      bear[3] = 33+30;
      bear[4] = 41+30;
    }

    bearCounter += bearInc;
    if(bearCounter>10){
      // time to move the bears
      bearCounter=0;
      strip.clear();
      for (i=0;i<MAXBEARS;i++){
        bear[i]-=2;
        if(bear[i]<3) bear[i]=79;
        if(i<bearNum){
          color0=getColor(bhue[i],1,bright);
        }else{
          color0=0;
        }
        strip.setPixelColor(bear[i],color0);
      }
    }
  }
  
  // ***************** mode 1, rainbow bears *******************************************************
  if(mode==1){

    if(old_mode!=1)  {
      bear[0] = 9+30;
      bear[1] = 17+30;
      bear[2] = 25+30;
      bear[3] = 33+30;
      bear[4] = 41+30;
    }

    bearCounter += bearInc;
    if(bearCounter>10){
      // time to move the bears
      bearCounter=0;
      strip.clear();
      for (i=0;i<MAXBEARS;i++){
        bear[i]-=2;
        if(bear[i]<3) bear[i]=79;
      }
    }
    for (i=0;i<MAXBEARS;i++){
      if(i<bearNum){
        index2=index-i*6;
        while(index2>360)index2-=360;
        while(index2<0)index2+=360;
        color0 = getColor(index2,1,bright);   
      }else{
        color0=0;
      }
      strip.setPixelColor(bear[i],color0);
    }
  }
        
  
  // ***************** mode 2, all RED for night light *******************************************************
  if(mode==2){
    
    if(old_mode!=2){
      strip.clear();
      delay_mode=200;
    }
    
    if(delay_mode>0){
      delay_mode--;
    }else{
      for(led=0;led<NUMLEDS;led+=2){
        color0 = getColor(120,1,bright);        
        strip.setPixelColor(led,color0);
      }
    }

  }
  // ***************** mode 3, all white for maximum light *******************************************************
  if(mode==3){
    
    if(old_mode!=3){
      strip.clear();
      delay_mode=200;
    }
    
    if(delay_mode>0){
      delay_mode--;
    }else{
      for(led=0;led<NUMLEDS;led+=1){
        strip.setPixelColor(led,0x00FFFFFF);
      }
    }
  }
  // ***************** mode 4, two fast bears 180 degrees apart *******************************************************
  if(mode==4){

    if(old_mode!=4)  {
      bear[0] = 79;
      bear[1] = 39;
      bear[2] = 0;
      bear[3] = 0;
      bear[4] = 0;
    }

    bearCounter += 5*bearInc;
    if(bearCounter>10){
      // time to move the bears
      bearCounter=0;
      strip.clear();
      for (i=0;i<MAXBEARS;i++){
        bear[i]-=2;
        if(bear[i]<3) bear[i]=79;
      }
    }
    for (i=0;i<2;i++){
      if(i<bearNum){
        index2=index;
        color0 = getColor(index2,1,bright);   
      }else{
        color0=0;
      }
      strip.setPixelColor(bear[i],color0);
    }


  }
  // ***************** two bears 180 with tracers *******************************************************
  if(mode==5){

    if(old_mode!=5)  {
      bear[0] = 79;
      bear[1] = 39;
      bear[2] = 0;
      bear[3] = 0;
      bear[4] = 0;
    }

    bearCounter += bearInc;
    if(bearCounter>10){
      // time to move the bears, negative marches forward
      bearCounter=0;
      for (i=0;i<MAXBEARS;i++){
        bear[i]-=2;
        if(bear[i]<3) bear[i]=79;
        trace[i]=bear[i];
        traceBright[i]=bright;
      }
    }
    
    traceCounter += traceInc;
    if(traceCounter>1){
      // time to move the traces, positive increment marches backward
      traceCounter=0;
      traceCounter2 += 1.0;
      if(traceCounter2>10){
        // reset trace to bear position and brightness
        traceCounter2=0;
        for (i=0;i<MAXBEARS;i++){
          trace[i]=bear[i];
          traceBright[i]=bright;
        }

      }
      color0=0;
      for (i=0;i<MAXBEARS;i++){
        strip.setPixelColor(trace[i],color0);   // turn off old trace
        trace[i]+=2;
        if(trace[i]>79) trace[i]=3;
        traceBright[i] *= 0.8;
      }
    }

    // clear the strip then draw the current bears and traces
    strip.clear();
    for (i=0;i<2;i++){
      if(i<bearNum){
        index2=index-i*180;
        while(index2>360)index2-=360;
        while(index2<0)index2+=360;
        color0 = getColor(index2,1,bright);   
        color1 = getColor(index2,1,traceBright[i]);   
      }else{
        color0=0;
        color1=0;
      }
      strip.setPixelColor(bear[i],color0);
      strip.setPixelColor(trace[i],color1);
    }


  }

  // ************** end of modes ********************************
  
  // Update the strip
  strip.show();             // Refresh strip
  //digitalWrite(10,LOW);     // De-select LEDS, selects accelerometer

}

//=================================================================================
