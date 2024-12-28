//==============================================================================
// Leds.cpp
//==============================================================================

#include "Leds.h"

#include "../RobotConfiguration.h"



//==============================================================================
// Construction

Leds::Leds() {

}

Leds::~Leds() {
}


//==============================================================================
// Pattern Managing

void Leds::SetLedPattern(Pattern pattern) {
    uint8_t buffer[1];
    buffer[0] = (int)pattern;
    m_arduino_i2c.WriteBulk(buffer, 1);
    //printf("I2C transaction %s\n", (ok ? "worked":"didn't work"));
}

void Leds::IncrementPattern() {
    m_pattern++;
    if (m_pattern>kTotalPatterns){
        m_pattern = 1;
    }
    SetLedPattern((Pattern)m_pattern);
}

