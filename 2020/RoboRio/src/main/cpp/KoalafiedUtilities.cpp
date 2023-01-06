//==============================================================================
// KoalafiedUtilities.cpp
//==============================================================================

#include "KoalafiedUtilities.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotConfiguration.h"
#include <iostream>

namespace RC = RobotConfiguration;


//==============================================================================
// Joystick Utilities

void KoalafiedUtilities::ClearJoystickButtonState(frc::Joystick* joystick)
{
	// Get the total number of buttons for the joystick. Note that the joystick state
	// comes from the DriverStation object.
	frc::DriverStation& driver_station = frc::DriverStation::GetInstance();
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

void KoalafiedUtilities::CalculateAndLogF(TalonSRX* controller, double speed_scale, const char* name) {
	// Calculate the motor output voltage as a fraction
	double motor_output = controller->GetMotorOutputVoltage()/controller->GetBusVoltage();

	// Get the speed in RPM and convert to the native units of encode counts
	// per 100ms time period (see TSSRM page 88).
	double speed_native = controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
	double speed = speed_native * speed_scale;

	// Calculate a feed forward gain (F) for this speed
	double F = motor_output * 1023.0/speed_native;

	// Log the values
	std::cout << name << ": Output " << motor_output << "  Speed " << speed << "  F " << F << "\n";
}

void KoalafiedUtilities::TuneDriveTalonSRX(TalonSRX* controller, const char* name, double drive, double max_rpm, bool close_loop) {
	// Do the tuning drive assuming a TalonSRX with a CTRE magnetic encoder
	TuneDriveTalon(controller, name, drive, max_rpm, close_loop, RC::kCtreEnocderCounts);
}

void KoalafiedUtilities::TuneDriveTalonFX(TalonFX* controller, const char* name, double drive, double max_rpm, bool close_loop) {
	// Do the tuning drive assuming a TalonFX and its built in encoder
	TuneDriveTalon(controller, name, drive, max_rpm, close_loop, RC::kTalonFXEnocderCounts);
}

void KoalafiedUtilities::TuneDriveTalon(BaseTalon* controller, const char* name, double drive, double max_rpm, bool close_loop, int encoder_per_rev) {
    if (close_loop) {
		// Close loop drive

		// Calculate the target RPM velocity and convert it to native units. Display both.
		double target_velocity_rpm = max_rpm * drive;
		double target_velocity_native = (target_velocity_rpm / 60.0) * encoder_per_rev * RC::kTalonTimeBaseS;
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (Native)", target_velocity_native);    
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (RPM)", target_velocity_rpm);    

		// Run the motor at the given drive in close loop
        controller->Set(ControlMode::Velocity, target_velocity_native);

        // Get the close loop error and display it in native units and RPM
        double closed_loop_error_native = controller->GetClosedLoopError(RC::kTalonPidIdx);
    	double closed_loop_error_rpm = (closed_loop_error_native / (encoder_per_rev * RC::kTalonTimeBaseS)) * 60.0;

        frc::SmartDashboard::PutNumber(std::string(name) + " CL Error (Native)", closed_loop_error_native);    
        frc::SmartDashboard::PutNumber(std::string(name) + " CL Error (RPM)", closed_loop_error_rpm);

		// Calculate an estimate for P based on 10% of the error, and display it
		double P = 0.0;
		if (closed_loop_error_native != 0) P = ((10.0/100.0) * 1023.0) / closed_loop_error_native;
        frc::SmartDashboard::PutNumber(std::string(name) + " P Esitmate", P);
    } else {
		// Open loop drive

		// Run the motor at the given drive in open loop. Display the drive so we can check sensor phase.
        controller->Set(ControlMode::PercentOutput, drive);
        frc::SmartDashboard::PutNumber(std::string(name) + " OL Drive", drive);		
	}

	// Get the current velocity and display it in native units and RPM
	double current_velocity_native = controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
   	double current_velocity_rpm = (current_velocity_native / (encoder_per_rev * RC::kTalonTimeBaseS)) * 60.0;
	frc::SmartDashboard::PutNumber(std::string(name) + " Velocity (Native)", current_velocity_native);    
	frc::SmartDashboard::PutNumber(std::string(name) + " Velocity (RPM)", current_velocity_rpm);    

	// For closed loop calculate and display a value for F
	if (!close_loop) {
        // Calculate the motor output voltage as a fraction
        double motor_output = controller->GetMotorOutputVoltage() / controller->GetBusVoltage();

        // Calculate a feed forward gain (F) for the current velocity and display it
        double F;
        if (current_velocity_native == 0) {
            F = 0;
        } else {
            F = motor_output * 1023.0/current_velocity_native;
        }
        frc::SmartDashboard::PutNumber(std::string(name) + " F estimate", F);
    }
}

double KoalafiedUtilities::TalonSRXCtreVelocityRpmToNative(double velocity_rpm) {
    return (velocity_rpm / 60.0) * RC::kCtreEnocderCounts * RC::kTalonTimeBaseS;
}

double KoalafiedUtilities::TalonSRXCtreVelocityNativeToRpm(double velocity_native) {
    return (velocity_native /  (RC::kCtreEnocderCounts * RC::kTalonTimeBaseS)) * 60.0;
}

double KoalafiedUtilities::TalonFXVelocityRpmToNative(double velocity_rpm) {
    return (velocity_rpm / 60.0) * RC::kTalonFXEnocderCounts * RC::kTalonTimeBaseS;
}

double KoalafiedUtilities::TalonFXVelocityNativeToRpm(double velocity_native) {
    return (velocity_native / (RC::kTalonFXEnocderCounts * RC::kTalonTimeBaseS)) * 60.0;
}
