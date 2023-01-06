//==============================================================================
// Kicker.cpp
//==============================================================================

#include "Kicker.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <iostream>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Kicker::Kicker()  {
}

Kicker::~Kicker() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Kicker::Setup() {
    std::cout << "Kicker::Setup()\n";

    m_kicker_double_solenoid = new frc::DoubleSolenoid(RC::kPneumaticsKickerForwardSolenoidId,
                                                       RC::kPneumaticsKickerReverseSolenoidId);
    SetStop();
}

void Kicker::Shutdown() {
    std::cout << "Kicker::Shutdown()\n";
}

void Kicker::Periodic() {      

}

//==============================================================================
// Operations

void Kicker::SetStop() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Kicker::SetShoot() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

void Kicker::SetOff() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Kicker::TestDriveKicker(frc::Joystick* joystick) {
    if (joystick->GetRawButton(RC::kJoystickAButton)) {
        SetShoot();
    } else if (joystick->GetRawButton(RC::kJoystickBButton)) {
        SetStop();
    } else {
        SetStop();
    }
}
