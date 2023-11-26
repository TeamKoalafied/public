//==============================================================================
// I2C Master Test
//
// This code is use to test controlling the RobotLed Arduino via a I2C link.
// It runs on a separate Arduino Nano with a single switch as input. When the
// switch is pushed is sends the next LED pattern command to the RobotLed Arduino.
//
// ALso https://www.instructables.com/I2C-between-Arduinos/
//==============================================================================

#include <Arduino.h>
#include <Wire.h>

#include "../../RobotLeds/src/Switch.h"

// I2C ID of the slave. This much be the same as the RobotLed Arduino.
#define I2C_SLAVE_ID 9

// Switch that is connected to a pin for user input
#define SWITCH_PIN  3
Switch g_switch(SWITCH_PIN);

// Current pattern index
int g_pattern_index = 1;

// Total number of patterns
const int TOTAL_PATTERNS = 10;


//===============================================================================
// Main Setup & Loop
//===============================================================================

void setup() {
    // Start the I2C Bus as Master. Set a timeout to prevent lockups.
    Wire.begin();
    Wire.setWireTimeout();

    // Setup the serial connection
    Serial.begin(115200);
    Serial.setTimeout(100);
    
    // Wait for serial to initialize.
    while (!Serial) { }

    // Display a starting message on the serial console
    Serial.println("");
    Serial.println("---------------------------------------------------------------");
    Serial.println("Starting Koalafies I2C Master Tester               Verion 1.0.0");
    Serial.println("---------------------------------------------------------------");

    // Update the switch as it may start in a closed position and we do
    // not want a spurious transition
    g_switch.Update();
}

void loop() {
    if (g_switch.Update() == Switch::DOWN) {
        // If the switch is pressed down increment the pattern index
        g_pattern_index++;
        if (g_pattern_index >= TOTAL_PATTERNS) g_pattern_index = 0;
        Serial.print("Switching to pattern ");
        Serial.println(g_pattern_index);

        // Send the pattern to the slave on the I2C bus
        Wire.beginTransmission(I2C_SLAVE_ID);
        Wire.write(g_pattern_index);
        Wire.endTransmission();
    }
}
