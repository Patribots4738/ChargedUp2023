#include "FastLED.h"
#include "Wire.h"

// How many leds in your strip?
#define NUM_LEDS 150
#define DATA_PIN 3
#define BRIGHTNESS 100

CRGB leds[NUM_LEDS];

int data = 0;
int index = 0;


void setup() {
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  randomSeed(analogRead(0));
  Serial.begin(9600);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {}

void receiveEvent()
{
  while (1 < Wire.available()) { // loop through the last data
    char c = Wire.read(); // receive byte as a character
  }
  data = Wire.read();
  if (data == 255) {data = -1;}
  if (data == 246) {data = -10;}
  index += data;
  if (index < 0){
    index = 0;
  }else if (index > NUM_LEDS){
    index = NUM_LEDS-1;
  }

  Serial.print(data);
  Serial.print(", ");
  Serial.println(index);

  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
//
  leds[index] = CRGB::White;

  FastLED.show();
}
