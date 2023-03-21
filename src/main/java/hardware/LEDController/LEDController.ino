#include "FastLED.h"
#include "Wire.h"

// How many leds in your strip?
#define NUM_LEDS 150
#define DATA_PIN 7 

#define FORWARD 0
#define BACKWARD 1
#define SLOW 250
#define MEDIUM 50
#define FAST 1
#define BRIGHTNESS 100

#define BELLYPAN_START_INDEX 0
#define BELLYPAN_END_INDEX 97
#define SPONSOR_PANEL_START_INDEX 0
#define SPONSOR_PANEL_END_INDEX 0
#define ARM_START_INDEX 98
#define ARM_END_INDEX 131
#define CLAW_START_INDEX 30
#define CLAW_END_INDEX 34

CRGB leds[NUM_LEDS];

int data = -1;

int theaterChaseRainbowIncrementer = 0;
int rainbowIncrementer = 0;
int bounceCenter = 0;

int bellyPanPattern = 0;
int sponsorPanelPattern = 0;
int armPattern = 0;
int clawPattern = 0;
int statusLED = false;

void setup() {
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.println("Recieved");
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  randomSeed(analogRead(0));
  Serial.begin(9600);
  FastLED.setBrightness(BRIGHTNESS);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  assignDataToSections();
  setBellyPan(bellyPanPattern);
//  setSponsorPanel(sponsorPanelPattern);
  setArm(armPattern);
//  setClaw(clawPattern);
  
}

void assignDataToSections(){
  if (data >= 0 && data <= 9){
    bellyPanPattern = data;
  }
  else if (data >= 10 && data <= 19){
    sponsorPanelPattern = data;
  }
  else if (data >= 20 && data <= 29){
    armPattern = data;
  }
  else if (data >= 30 && data <= 39){
    clawPattern = data;
  }  
  
}

// Changes all LEDS to given color
void allColor(int startIndex, int endIndex, CRGB c){
  for (int i = startIndex; i < endIndex; i++){
    leds[i] = c;
  }
  FastLED.show();
  
}

// Set the pattern of the LEDs to have a lighter color bounce left to right
void bounce(int startIndex, int endIndex, CRGB c, int speed) { // TODO directionk
  if (bounceCenter < endIndex) {
    for (int i = startIndex; i < endIndex; i++) {
      // Set the 2nd led to a lighter version of param c
      if (i > bounceCenter - (endIndex-startIndex/10) && i < bounceCenter + (endIndex-startIndex/10)) {
        double brightness = (constrain(1/abs(i - bounceCenter), 1, 4));
        leds[i] = CRGB(constrain(c.r*brightness, 0, 255), constrain(c.g*brightness, 0, 255), constrain(c.b*brightness, 0, 255));
      }
      leds[i] = c;
    }
    FastLED.show();
    delay(speed);
    bounceCenter += 1;
  }
  else {
    bounceCenter = startIndex;
  }

}


// Flashes given color
// If c==NULL, random color flash
void flash(int startIndex, int endIndex, CRGB c, int speed){
    if(c){
      allColor(startIndex, endIndex, c);
    }
    else{
      allColor(startIndex, endIndex, randomColor());
    }
    delay(speed);
    allColor(startIndex, endIndex, CRGB::Black);
    delay(speed);
  
}

void rainbow(int startIndex, int endIndex, int cycles, int speed){ // TODO directionk
    if (rainbowIncrementer < 256){
      for (int i = startIndex; i < endIndex; i++){
        leds[i] = CHSV(i-(rainbowIncrementer*2), 255, 255);
      }
//      fill_rainbow( leds, endIndex, rainbowIncrementer, 7);
      FastLED.show();
      delay(speed);
      rainbowIncrementer += 1;
    }
    else {
      rainbowIncrementer = 0;
    }
  
}

// Theater-style crawling lights
void theaterChase(int startIndex, int endIndex, CRGB c, int speed){ // TODO direction
  for (int q=0; q < 3; q++) {
    for (int i=startIndex; i < endIndex; i=i+3) {
      int pos = i+q;
      leds[pos] = c;    //turn every third pixel on
    }
    FastLED.show();

    delay(speed);

    for (int i=startIndex; i < endIndex; i=i+3) {
      leds[i+q] = CRGB::Black;        //turn every third pixel off
    }
  }
  
}

// Theater-style crawling lights with rainbow effect
void theaterChaseRainbow(int startIndex, int endIndex, int speed){ // TODO direction, duration
  if (theaterChaseRainbowIncrementer < 256) {
    for (int q=0; q < 3; q++) {
      for (int i=startIndex; i < endIndex; i=i+3) {
        int pos = i+q;
        leds[pos] = Wheel( (i+theaterChaseRainbowIncrementer) % 255);    //turn every third pixel on
      }
      FastLED.show();

      delay(speed);

      for (int i=startIndex; i < endIndex; i=i+3) {
        leds[i+q] = CRGB::Black;  //turn every third pixel off
      }
    }
    theaterChaseRainbowIncrementer += 1;
  }
  else {
    theaterChaseRainbowIncrementer = 0;
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
CRGB Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return CRGB(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return CRGB(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return CRGB(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

CRGB randomColor(){
  return Wheel(random(256)); 
}

void receiveEvent()
{
  while (1 < Wire.available()) { // loop through the last data
    char c = Wire.read(); // receive byte as a character
//    Serial.print(c);
  }
  data = Wire.read();
//  Serial.println(data);

  digitalWrite(LED_BUILTIN, true);
  statusLED = !statusLED;
}


//TODO: add indexs to all used methods

// ----------------------------------------------------------------------------------------------------------------
// Belly Pan          0                1                 2          3       4    5      6      7       8      9   >>
//                 Rainbow,  theaterChaseRainbow, theaterChase,  FlashRed, Red, Blue, Green, Purple, Yellow, Off
//
// Sponsor Panel >>  10              11                12          13      14   15     16     17      28     29   >>
//                 Rainbow,  theaterChaseRainbow, theaterChase,   NULL,    Red, Blue, Green, Purple, Yellow, Off
//
// Arm           >>  20              21                22          23      24   25     26     27      28     29   >>
//                 Rainbow,  theaterChaseRainbow, theaterChase,   NULL,    Red, Blue, Green, Purple, Yellow, Off
//
// Claw          >>  30              31                32          33      34   35     36     37      38     39
//                 Rainbow,  theaterChaseRainbow, theaterChase,   NULL,    Red, Blue, Green, Purple, Yellow, Off
// ----------------------------------------------------------------------------------------------------------------

void setBellyPan(int pattern){
  switch(pattern){
    case -1:
      allColor(0, NUM_LEDS, CRGB::Black);
      
    case 0: // Rainbow
      rainbow(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, 1, FAST);
      break;

    case 1: //theaterChaseRainbow
      theaterChaseRainbow(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, MEDIUM);
      break;

    case 2: //theaterChase
      theaterChase(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, randomColor(), MEDIUM);
      break;
      
    case 3: // FlashRed
      flash(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Red, SLOW);
      break;
      
    case 4: // 
      allColor(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Red);
      break;
      
    case 5: // Blue
      bounce(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Blue, MEDIUM);
      break;
      
    case 6: // Green
      allColor(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Green);
      break;
      
    case 7: // Purple
      allColor(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Purple);
      break;
      
    case 8: // Yellow
      allColor(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Yellow);
      break;

    case 9: // Off
      allColor(BELLYPAN_START_INDEX, BELLYPAN_END_INDEX, CRGB::Black);
      break;
  }
}

void setSponsorPanel(int pattern){
  switch(pattern){
    case 10: // Rainbow
      rainbow(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, 1, FAST);
      break;

    case 11: //theaterChaseRainbow
      theaterChaseRainbow(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, MEDIUM);
      break;

    case 12: //theaterChase
      theaterChase(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, randomColor(), MEDIUM);
      break;
      
    case 13: // NULL
      break;
      
    case 14: // Red
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Red);
      break;
      
    case 15: // Blue
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Blue);
      break;
      
    case 16: // Green
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Green);
      break;
      
    case 17: // Purple
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Purple);
      break;
      
    case 18: // Yellow
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Yellow);
      break;

    case 19: // Off
      allColor(SPONSOR_PANEL_START_INDEX, SPONSOR_PANEL_END_INDEX, CRGB::Black);
      break;
  }
}

void setArm(int pattern){
    switch(pattern){
    case 20: // Rainbow
      rainbow(ARM_START_INDEX, ARM_END_INDEX, 1, FAST);
      break;

    case 21: //theaterChaseRainbow
      theaterChaseRainbow(ARM_START_INDEX, ARM_END_INDEX, MEDIUM);
      break;

    case 22: //theaterChase
      theaterChase(ARM_START_INDEX, ARM_END_INDEX, randomColor(), MEDIUM);
      break;
      
    case 23: // NULL
      flash(ARM_START_INDEX, ARM_END_INDEX, CRGB::Red, SLOW);
      break;
      
    case 24: // Red
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Red);
      break;
      
    case 25: // Blue
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Blue);
      break;
      
    case 26: // Green
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Green);
      break;
      
    case 27: // Purple
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Purple);
      break;
      
    case 28: // Yellow
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Yellow);
      break;

    case 29: // Off
      allColor(ARM_START_INDEX, ARM_END_INDEX, CRGB::Black);
      break;
  }
}
void setClaw(int pattern){
    switch(pattern){
    case 30: // Rainbow
      rainbow(CLAW_START_INDEX, CLAW_END_INDEX, 1, FAST);
      break;

    case 31: //theaterChaseRainbow
      theaterChaseRainbow(CLAW_START_INDEX, CLAW_END_INDEX, MEDIUM);
      break;

    case 32: //theaterChase
      theaterChase(CLAW_START_INDEX, CLAW_END_INDEX, randomColor(), MEDIUM);
      break;
      
    case 33: // NULL
      break;
      
    case 34: // Red
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Red);
      break;
      
    case 35: // Blue
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Blue);
      break;
      
    case 36: // Green
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Green);
      break;
      
    case 37: // Purple
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Purple);
      break;
      
    case 38: // Yellow
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Yellow);
      break;

    case 39: // Off
      allColor(CLAW_START_INDEX, CLAW_END_INDEX, CRGB::Black);
      break;
  }
}