//==============================================================================
// KoalafiedUtilities.cpp
//==============================================================================

#include "KoalafiedUtilities.h"
#include <frc/DriverStation.h>
#include "RobotConfiguration.h"
#include <iostream>


//==============================================================================
// Joystick Utilities

void KoalafiedUtilities::ClearJoystickButtonState(frc::Joystick* joystick)
{
	// Get the total number of buttons for the joystick. Note that the joystick state
	// comes from the DriverStation object.
	DriverStation& driver_station = DriverStation::GetInstance();
	int joystick_port = joystick->GetPort();
	int total_buttons = driver_station.GetStickButtonCount(joystick_port);

	// Query the 'pressed' and 'released' state for all buttons as this will reset them.
	for (int i = 1; i <= total_buttons; i++) {
		driver_station.GetStickButtonPressed(joystick_port, i);
		driver_station.GetStickButtonReleased(joystick_port, i);
	}
}

double KoalafiedUtilities::PowerAdjust(double value, double power) {
	if (value == 0.0) return 0.0;
    if (value >= 0.0) return pow(value, power);
    else              return -pow(-value, power);
}


//==========================================================================
// Talon SRX Utilities

void KoalafiedUtilities::CalculateAndLogF(TalonSRX* controller, double speed_scale, const char* name)
{
	// Calculate the motor output voltage as a fraction
	double motor_output = controller->GetMotorOutputVoltage()/controller->GetBusVoltage();

	// Get the speed in RPM and convert to the native units of encode counts
	// per 100ms time period (see TSSRM page 88).
	double speed_native = controller->GetSelectedSensorVelocity(RobotConfiguration::kTalonPidIdx);
	double speed = speed_native * speed_scale;

	// Calculate a feed forward gain (F) for this speed
	double F = motor_output * 1023.0/speed_native;

	// Log the values
	std::cout << name << ": Output " << motor_output << "  Speed " << speed << "  F " << F << "\n";
}

