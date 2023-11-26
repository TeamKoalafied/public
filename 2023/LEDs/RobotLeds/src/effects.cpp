#include "effects.h"
#include <FastLED.h>

// void Effects::rainbow(CRGB* leds, int NUM_LEDS, int delay, int chaseSize, int chaseSpeed) {
//   static uint8_t startIndex = 0;
//   startIndex = (startIndex + chaseSpeed) % NUM_LEDS;

//   // Fill the LED strip with a rainbow pattern
//   for (int i = 0; i < NUM_LEDS; i++) {
//     uint8_t hue = map((i + startIndex) % NUM_LEDS, 0, NUM_LEDS, 0, 255);
//     leds[i] = CHSV(hue, 255, 255);
//   }

//   // Shift the rainbow pattern by chaseSize positions
//   for (int i = 0; i < chaseSize; i++) {
//     leds[(startIndex + i) % NUM_LEDS] = CRGB::Black;
//   }
// 	FastLED.delay(delay);
// }

void Effects::chase(CRGB *leds, int NUM_LEDS, int cometDelay, int cometSize, int cometSpeed, int cometFade, CRGB cometColor)
{
  static int cometPosition = 0;
  static unsigned long previousMillis = 0;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= cometDelay)
  {
    previousMillis = currentMillis;

    fadeToBlackBy(leds, NUM_LEDS, cometFade);

    for (int i = 0; i < cometSize; i++)
    {
      int index = (cometPosition - i + NUM_LEDS) % NUM_LEDS;
      leds[index] = cometColor;
    }
    cometPosition = (cometPosition + cometSpeed) % NUM_LEDS;

    FastLED.show();
  }
}

void Effects::pulse(CRGB *leds, int NUM_LEDS, int pulseDelay, int minBrightness, int maxBrightness, int pulseIncrement, CRGB color)
{
  static int brightness = minBrightness;
  static bool increasing = true;
  static unsigned long previousMillis = 0;

  // Get the current time
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= pulseDelay)
  {
    previousMillis = currentMillis;

    if (increasing)
    {
      brightness += pulseIncrement;
      if (brightness >= maxBrightness)
      {
        brightness = maxBrightness;
        increasing = false;
      }
    }
    else
    {
      brightness -= pulseIncrement;
      if (brightness <= minBrightness)
      {
        brightness = minBrightness;
        increasing = true;
      }
    }
  }

  fill_solid(leds, NUM_LEDS, CRGB(color.r * brightness / 255, color.g * brightness / 255, color.b * brightness / 255));
}

void Effects::solid(CRGB *leds, int NUM_LEDS, CRGB color, int brightness)
{
  fill_solid(leds, NUM_LEDS, CRGB(color.r * brightness / 255, color.g * brightness / 255, color.b * brightness / 255));
}
// void Effects::bounceChase(CRGB* leds, int NUM_LEDS, int cometDelay, int cometSize, int cometSpeed, int cometFade, CRGB cometColor) {
//    static int cometPosition = 0;
//   static int cometDirection = 1;

//   fadeToBlackBy(leds, NUM_LEDS, cometFade);

//   for (int i = 0; i < cometSize; i++) {
//     int index = cometPosition + i;
//     if (cometDirection == -1) {
//       index = NUM_LEDS - index - 1;
//     }
//     index = (index + NUM_LEDS) % NUM_LEDS;
//     leds[index] = cometColor;
//   }

//   cometPosition += cometSpeed * cometDirection;

//   if (cometPosition >= NUM_LEDS - 1) {
//     cometPosition = NUM_LEDS - 1;
//     cometDirection = -1;
//   } else if (cometPosition < 0) {
//     cometPosition = 0;
//     cometDirection = 1;
//   }

//   FastLED.delay(cometDelay);
// }

void Effects::rainbowCycle(CRGB *leds, int NUM_LEDS, int cycleDelay)
{
  static uint8_t hue = 0;
  static unsigned long loopDuration = 0;
  loopDuration = (loopDuration + 1) % ((cycleDelay));
  if (loopDuration == 0)
  {
    hue++;
  }
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
  }
}
// void Effects::sparkle(CRGB *leds, int NUM_LEDS, int sparkleCount, CRGB sparkleColor, int sparkleDelay)
// {
//   static unsigned long loopDuration = 0;
//   loopDuration = (loopDuration + 1) % ((sparkleDelay)*2);
//   if (loopDuration == 0)
//   {
//     while (sparkleCount --> 0)
//     {
//       int index = random(NUM_LEDS);
//       leds[index] = sparkleColor;
//     }
//   }
//   if (loopDuration == sparkleDelay)
//   {
//     while (NUM_LEDS --> 0)
//     {
//       leds[NUM_LEDS] = CRGB::Black;
//     }
//   }
// }
void Effects::colorWipe(CRGB *leds, int NUM_LEDS, int wipeDelay, CRGB color, int wipeSize)
{
  static unsigned long loopDuration = 0;
  loopDuration = (loopDuration + wipeSize) % ((NUM_LEDS)*2);
  int i = loopDuration;
  if (i < NUM_LEDS)
  {
    for (int j = 0; j < i + wipeSize; j++)
    {
      if (j < NUM_LEDS)
      {
        leds[j] = color;
      }
    }
  }
  if (i >= NUM_LEDS)
  {
    i -= NUM_LEDS;
    for (int j = i; j < NUM_LEDS; j++)
    {
      if (j < NUM_LEDS)
      {
        leds[j] = color;
      }
    }
    for (int j = i; j < i + wipeSize; j++)
    {
      if (j < NUM_LEDS)
      {
        leds[j] = CRGB::Black;
      }
    }
  }
}
void Effects::pride(CRGB *leds, int NUM_LEDS)
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88(87, 220, 250);
  uint8_t brightdepth = beatsin88(341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88(203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16; // gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis;
  sLastMillis = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88(400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16 += brightnessthetainc16;
    uint16_t b16 = sin16(brightnesstheta16) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV(hue8, sat8, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;

    nblend(leds[pixelnumber], newcolor, 64);
  }
}
void Effects::theaterChase(CRGB *leds, int NUM_LEDS, int chaseDelay, int chaseSize, CRGB color)
{
  static unsigned long loopDuration = 0;
  loopDuration = (loopDuration + 1) % (chaseDelay * chaseSize);
  int chaseOffset = (int)loopDuration / chaseDelay;
  int lastChaseOffset = (int)(loopDuration - 1) / chaseDelay;
  for (int i = 0; i < NUM_LEDS; i += chaseSize)
  {
    leds[i + chaseOffset] = color;
  }
  if (lastChaseOffset != chaseOffset)
  {
    for (int i = 0; i < NUM_LEDS; i += chaseSize)
    {
      leds[i + lastChaseOffset] = CRGB::Black;
    }
  }
  FastLED.show();
}
