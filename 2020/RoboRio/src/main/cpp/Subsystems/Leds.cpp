//==============================================================================
// Leds.cpp
//==============================================================================

#include "Leds.h"

#include "../RobotConfiguration.h"



//==============================================================================
// Construction

Leds::Leds() :
    TSingleton<Leds>(this),
    frc::Subsystem("Leds") {

}

Leds::~Leds() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Leds::InitDefaultCommand() {
    // No default command
}

//Pattern Managing
void Leds::SetLedPattern(int pattern) {
	uint8_t buffer[2];
	buffer[0] = 'L';
	buffer[1] = pattern;
	bool ok = m_arduino_i2c.WriteBulk(buffer, 2);
	printf("I2C transaction %s\n", (ok ? "worked":"didn't work"));
}

void Leds::IncrementPattern() {
	m_pattern++;
	if (m_pattern>kTotalPatterns){
		m_pattern = 1;
	}
	SetLedPattern(m_pattern);
}

//==============================================================================
// Setup and Shutdown

void Leds::Setup() {
    printf("Leds::Setup()\n");

}

void Leds::Shutdown() {
    printf("Leds::Shutdown()\n");

}
