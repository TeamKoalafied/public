//===============================================================================
// Koalafied Robot LED Controller
//
// This code runs on an Arduino Nano and handles displaying patterns on addressable
// LED strips. The pattern to display is set via an I2C connection. This code is
// and I2C slave and normally the RoboRIO is the I2C master, but there is also
// test code (see I2CMaster) so that an Arduino Nano can be the master for testing
// purposes. 
//
// See Subsystems/Leds in RoboRIO code.
//===============================================================================

#include <Arduino.h>

#include "FastLED.h"
#include <Wire.h>

#include "Switch.h"

#include "effects.h"
#include "colors.h"


#define I2C_SLAVE_ID 9

// Pin, chip type and colour format for the LED strip
#define LED_PIN     4
#define LED_PIN2     3
#define LED_TYPE  WS2812
#define COLOR_ORDER GRB

// Number of LEDs per segment
const int SEGMENT_LEDS = 32;
const int SEGMENTS = 4;
// 30*4=120

// Buffer containing the colours for the LED strip
const int TOTAL_LEDS = (SEGMENT_LEDS * SEGMENTS);
CRGB leds[TOTAL_LEDS];

// Each LED segment is a pointer to a section of the LED buffer
CRGB* ledSegments[4] = {leds, leds + SEGMENT_LEDS, leds + (SEGMENT_LEDS * 2), leds + (SEGMENT_LEDS * 3)};

// Number of times to update the LED animation per second
#define UPDATES_PER_SECOND 20

// Palette to use for the LED pattern
CRGBPalette16 currentPalette;

// Type of blending to use for the LED pattern
TBlendType currentBlending;


// Position for LED animation
uint8_t g_start_index = 0;

// Brightness to use for the LEDs
int g_led_brightness = 200;

// Switch that is connected to a pin for user input
#define SWITCH_PIN  5
Switch g_switch(SWITCH_PIN);

// Current pattern index
int g_pattern_index = 0;

// Total number of patterns
const int TOTAL_PATTERNS = 12;


//===============================================================================
// LED Control Functions
//===============================================================================

// Setup the LEDs for a given pattern index
//
// pattern - Pattern index to set up
// void SetLedPattern(int pattern)
// {
//     Serial.print("Switching to pattern ");
//     Serial.println(g_pattern_index);

//     switch (pattern) { 
//         // case 0:
//         //     drawComet(leds, SEGMENT_LEDS, 5, 5, 5, 50, CRGB::Yellow);
//         //     break;
//         // case 1:
//         //     drawComet(leds, SEGMENT_LEDS, 5, 5, 5, 50, CRGB::Purple);
//         //     break;
//         // case 2:
//         //     rainbow();
//         //     break;
//         // case 3:
//         //     fill_solid( currentPalette, 16, CRGB::Green);
//         //     break;
//         // case 4:
//         //     fill_solid( currentPalette, 16, CRGB::Blue);
//         //     break;
//         // case 5:
//         //     fill_solid( currentPalette, 16, CRGB::Black);
//         //     currentPalette[0] = CRGB::White;
//         //     currentPalette[4] = CRGB::White;
//         //     currentPalette[8] = CRGB::White;
//         //     currentPalette[12] = CRGB::White;
//         //     break;
//         // case 6:
//         //     fill_solid( currentPalette, 16, CRGB::Black);
//         //     currentPalette[0] = CRGB::Red;
//         //     currentPalette[4] = CRGB::Red;
//         //     currentPalette[8] = CRGB::Red;
//         //     currentPalette[12] = CRGB::Red;
//         //     break;
//         // case 7:
//         //     fill_solid( currentPalette, 16, CRGB::Black);
//         //     currentPalette[0] = CRGB::Green;
//         //     currentPalette[4] = CRGB::Green;
//         //     currentPalette[8] = CRGB::Green;
//         //     currentPalette[12] = CRGB::Green;
//         //     break;
//         // case 8:
//         //     fill_solid( currentPalette, 16, CRGB::Black);
//         //     currentPalette[0] = CRGB::Blue;
//         //     currentPalette[4] = CRGB::Blue;
//         //     currentPalette[8] = CRGB::Blue;
//         //     currentPalette[12] = CRGB::Blue;
//         //     break;
//         // case 9:
//         //     currentPalette = RainbowColors_p;
//         //     break;
//     }
// }

// Perform the initial setup of the LEDs. This is down onces at start up. 
void SetupLeds()
{
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, TOTAL_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  g_led_brightness );
    
    currentBlending = LINEARBLEND;
    
    Serial.println("SetupLeds() Complete");
}

//===============================================================================
// Effect Distributor/Mapping
// Please do not change !!
//===============================================================================
void UpdateLedAnimation()
{
    g_start_index++;
    for( int i = 0; i < TOTAL_LEDS; i++) {
        leds[i] = currentPalette.entries[(i + g_start_index) % 16];
    }
    switch (g_pattern_index) {
        case 0: 
            // Actively clears LEDs
            FastLED.clear(true);
            break;
           
        case 1: 
            Effects::pulse(leds, TOTAL_LEDS, 8,  10, 255, 5, Colors::Red);
            

            break;
         
        case 2: 
            Effects::pulse(leds, TOTAL_LEDS, 8,  10, 255, 5, Colors::Blue);
            break;
           
        case 3: 
            Effects::pulse(leds, TOTAL_LEDS, 0,  30, 255, 5, Colors::Cube);
            break;
            
        case 4: 
            Effects::pulse(leds, TOTAL_LEDS, 0,  30, 255, 5, Colors::Cone);
            break;
           
        case 5: 
            Effects::chase(leds, TOTAL_LEDS, 0,  40, 10, 10, Colors::Cube);
            break;
           
        case 6: 
            Effects::chase(leds, TOTAL_LEDS, 0,  40, 10, 10, Colors::Cone);
            break;
           
        case 7:
            Effects::solid(leds, TOTAL_LEDS, Colors::Cube, 255);
            break;
            
        case 8:
            Effects::solid(leds, TOTAL_LEDS, Colors::Cone, 255);
            break;
            
        case 9:
            Effects::solid(leds, TOTAL_LEDS, Colors::Red, 255);
            break;  

        case 10:
            Effects::solid(leds, TOTAL_LEDS, Colors::Green, 255);
            break;  

        case 11:
            Effects::pride(leds, TOTAL_LEDS);
            break;
    }
}


//===============================================================================
// I2C Event Handler
//===============================================================================

// Handle data being receive on the I2C bus
//
// bytes - number of bytes received
void I2CReceiveEvent(int bytes)
{
    // Get a single byte from the I2C bus which is the index of the pattern to display
    g_pattern_index = Wire.read();
    Serial.println("I2CReceiveEvent() - Pattern Index Update: " + String(g_pattern_index));
    // Clip the pattern to the legal range in case of an error
    if (g_pattern_index < 0) g_pattern_index = 0;
    if (g_pattern_index >= TOTAL_PATTERNS) g_pattern_index = TOTAL_PATTERNS - 1;

}


//===============================================================================
// Main Setup & Loop
//===============================================================================

void setup() {
    // Setup the serial connection
    Serial.begin(115200);
    Serial.setTimeout(100);
    
    // Wait for serial to initialize.
    long count = 100000;
    while (!Serial && count-- > 0) { }

    // Display a starting message on the serial console
    Serial.println("");
    Serial.println("---------------------------------------------------------------");
    Serial.println("Starting Koalafies Robot LED Controller            Verion 1.1.5");
    Serial.println("---------------------------------------------------------------");

    // Start the I2C Bus as a Slave and set the handler for receiving data
    Wire.begin(I2C_SLAVE_ID); 
    Wire.onReceive(I2CReceiveEvent);

    // Update the switch as it may start in a closed position and we do
    // not want a spurious transition
    g_switch.Update();

    // Perform initial led setup
    SetupLeds();

}



void loop() {
    // If the button is pressed down, increment the pattern being displayed. This is just
    // helpful for simple testing of pattern while not connect to an I2C master.
    if (g_switch.Update() == Switch::DOWN) {
        g_pattern_index++;
        if (g_pattern_index >= TOTAL_PATTERNS) g_pattern_index = 0;
        Serial.println("Switch Pressed - Pattern Index Update: " + String(g_pattern_index));
    }

    UpdateLedAnimation();

    
    FastLED.show();  
}


