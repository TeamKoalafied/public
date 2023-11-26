#ifndef EFFECTS_H
#define EFFECTS_H
#include <FastLED.h>

#define LED_TYPE WS2812B

class Effects {
public:
  
  static void pulse(CRGB* leds, int NUM_LEDS, int pulseDelay, int minBrightness, int maxBrightness, int pulseIncrement, CRGB color);
	static void chase(CRGB* leds, int NUM_LEDS, int cometDelay, int cometSize, int cometSpeed, int cometFade,  CRGB cometColor);
	// static void bounceChase(CRGB* leds, int NUM_LEDS, int cometDelay, int cometSize, int cometSpeed, int cometFade, CRGB cometColor);
	static void rainbowCycle(CRGB* leds, int NUM_LEDS, int cycleDelay);
	// static void sparkle(CRGB* leds, int NUM_LEDS, int sparkleCount, CRGB sparkleColor, int sparkleDelay);
	static void colorWipe(CRGB* leds, int NUM_LEDS,int wipeDelay, CRGB color, int wipeSize);
	static void theaterChase(CRGB* leds, int NUM_LEDS, int chaseDelay, int chaseSize, CRGB color);
	static void solid(CRGB* leds, int NUM_LEDS, CRGB color, int brightness);
	static void pride(CRGB* leds, int NUM_LEDS);
};

#endif
