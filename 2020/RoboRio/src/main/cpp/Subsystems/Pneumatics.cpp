//==============================================================================
// Leds.cpp
//==============================================================================

#include "Pneumatics.h"

#include "../RobotConfiguration.h"

#include <frc/AnalogInput.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Pneumatics::Pneumatics() :
    TSingleton<Pneumatics>(this),
    frc::Subsystem("Pneumatics") {

    m_compressor = NULL;
    m_pressure_guage_input = NULL;
}

Pneumatics::~Pneumatics() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Pneumatics::InitDefaultCommand() {
    // No default command
}

void Pneumatics::Periodic() {
    // Get the 12-bit value from the air pressure sensor and convert it to a pressure
    // in PSI.
    // See http://www.revrobotics.com/content/docs/REV-11-1107-DS.pdf for details
    
    // int air_pressure_sensor = m_pressure_guage_input->GetValue();
    // double air_pressure_psi = 250.0 * (air_pressure_sensor/1023.0) - 25.0;
    //frc::SmartDashboard::PutNumber("Air Pressure", air_pressure_psi);
}


//==============================================================================
// Setup and Shutdown

void Pneumatics::Setup() {
    printf("Pneumatics::Setup()\n");
    m_compressor = new frc::Compressor(RobotConfiguration::kPneumaticsControlModuleId);
    m_compressor->SetClosedLoopControl(true);
//    m_compressor->SetClosedLoopControl(false);

    m_pressure_guage_input = new frc::AnalogInput(RC::kAnalogInPressureSensorId);
}

void Pneumatics::Shutdown() {
    printf("Pneumatics::Shutdown()\n");

    delete m_compressor;
    m_compressor = NULL;

    delete m_pressure_guage_input;
    m_pressure_guage_input = NULL;
}
