//==============================================================================
// Leds.cpp
//==============================================================================

#include "Pneumatics.h"

#include "../RobotConfiguration.h"

#include <Compressor.h>


//==============================================================================
// Construction

Pneumatics::Pneumatics() :
    TSingleton<Pneumatics>(this),
    frc::Subsystem("Pneumatics") {

}

Pneumatics::~Pneumatics() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Pneumatics::InitDefaultCommand() {
    // No default command
}


//==============================================================================
// Setup and Shutdown

void Pneumatics::Setup() {
    printf("Pneumatics::Setup()\n");
    m_compressor = new frc::Compressor(RobotConfiguration::kPneumaticsControlModuleId);
    m_compressor->SetClosedLoopControl(true);
}

void Pneumatics::Shutdown() {
    printf("Pneumatics::Shutdown()\n");

    delete m_compressor;
    m_compressor = NULL;
}
