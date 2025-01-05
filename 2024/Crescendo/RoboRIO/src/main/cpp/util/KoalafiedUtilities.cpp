//==============================================================================
// KoalafiedUtilities.cpp
//==============================================================================

#include "KoalafiedUtilities.h"

#include "../RobotConfiguration.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

namespace RC = RobotConfiguration;


//==============================================================================
// Joystick Utilities

void KoalafiedUtilities::ClearJoystickButtonState(frc::Joystick* joystick)
{
    // Get the total number of buttons for the joystick. Note that the joystick state
    // comes from the DriverStation object.
    int joystick_port = joystick->GetPort();
    int total_buttons = frc::DriverStation::GetStickButtonCount(joystick_port);

    // Query the 'pressed' and 'released' state for all buttons as this will reset them.
    for (int i = 1; i <= total_buttons; i++) {
        frc::DriverStation::GetStickButtonPressed(joystick_port, i);
        frc::DriverStation::GetStickButtonReleased(joystick_port, i);
    }
}

double KoalafiedUtilities::PowerAdjust(double value, double power) {
    if (value == 0.0) return 0.0;
    if (value >= 0.0) return pow(value, power);
    else              return -pow(-value, power);
}


//==========================================================================
// Talon SRX & Talon FX Unit Conversions

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

double KoalafiedUtilities::TalonVelocityRpmToNative(double velocity_rpm, int encoder_counts) {
    return (velocity_rpm / 60.0) * encoder_counts * RC::kTalonTimeBaseS;
}

double KoalafiedUtilities::TalonVelocityNativeToRpm(double velocity_native, int encoder_counts) {
    return (velocity_native / (encoder_counts * RC::kTalonTimeBaseS)) * 60.0;
}


//==========================================================================
// Talon SRX & Talon FX Tuning

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
    TuneDriveTalon(controller, name, drive, max_rpm, close_loop, RC::kCtreEnocderCounts);
}

void KoalafiedUtilities::TuneDriveTalonFX(ctre::phoenix6::hardware::TalonFX* controller, const char* name, double drive, double max_rpm, bool close_loop) {
    if (close_loop) {
        // Close loop drive

        // Calculate the target RPM velocity and convert it to native units. Display both.
        units::revolutions_per_minute_t target_velocity_rpm = units::revolutions_per_minute_t(max_rpm * drive);
        double target_velocity_native = TalonVelocityRpmToNative(target_velocity_rpm.value(), RC::kTalonFXEnocderCounts);
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (Native)", target_velocity_native);    
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (RPM)", target_velocity_rpm.value());    

        // Run the motor at the given drive in close loop
        controller->SetControl(ctre::phoenix6::controls::VelocityDutyCycle(target_velocity_rpm));

        // Get the close loop error and display it in native units and RPM
        double closed_loop_error_native = controller->GetClosedLoopError().GetValueAsDouble();
        double closed_loop_error_rpm = TalonVelocityNativeToRpm(closed_loop_error_native, RC::kTalonFXEnocderCounts);
        frc::SmartDashboard::PutNumber(std::string(name) + " CL Error (Native)", closed_loop_error_native);    
        frc::SmartDashboard::PutNumber(std::string(name) + " CL Error (RPM)", closed_loop_error_rpm);

        // Calculate an estimate for P based on 10% of the error, and display it
        double P = 0.0;
        if (closed_loop_error_native != 0) P = ((10.0/100.0) * 1023.0) / closed_loop_error_native;
        frc::SmartDashboard::PutNumber(std::string(name) + " P Esitmate", P);
    } else {
        // Open loop drive

        // Run the motor at the given drive in open loop. Display the drive so we can check sensor phase.
        controller->SetControl(ctre::phoenix6::controls::DutyCycleOut(drive, false));
        frc::SmartDashboard::PutNumber(std::string(name) + " OL Drive", drive);		
    }

    // Get the current velocity and display it in native units and RPM
    // double current_velocity_native = controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    // double current_velocity_rpm = TalonVelocityNativeToRpm(current_velocity_native, RC::kTalonFXEnocderCounts);
    units::revolutions_per_minute_t current_velocity_rpm = controller->GetRotorVelocity().GetValue();
    double current_velocity_native = TalonFXVelocityRpmToNative(current_velocity_rpm.value());
    frc::SmartDashboard::PutNumber(std::string(name) + " Velocity (Native)", current_velocity_native);    
    frc::SmartDashboard::PutNumber(std::string(name) + " Velocity (RPM)", current_velocity_rpm.value());    

    // For closed loop calculate and display a value for F
    if (!close_loop) {
        // Calculate the motor output voltage as a fraction
        double motor_output = controller->GetMotorVoltage().GetValue() / controller->GetSupplyVoltage().GetValue();

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

void KoalafiedUtilities::TuneDriveTalon(BaseTalon* controller, const char* name, double drive,
                                       double max_rpm, bool close_loop, int encoder_counts) {
    if (close_loop) {
        // Close loop drive

        // Calculate the target RPM velocity and convert it to native units. Display both.
        double target_velocity_rpm = max_rpm * drive;
        double target_velocity_native = TalonVelocityRpmToNative(target_velocity_rpm, encoder_counts);
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (Native)", target_velocity_native);    
        frc::SmartDashboard::PutNumber(std::string(name) + " Target Velocity (RPM)", target_velocity_rpm);    

        // Run the motor at the given drive in close loop
        controller->Set(ControlMode::Velocity, target_velocity_native);

        // Get the close loop error and display it in native units and RPM
        double closed_loop_error_native = controller->GetClosedLoopError(RC::kTalonPidIdx);
        double closed_loop_error_rpm = TalonVelocityNativeToRpm(closed_loop_error_native, encoder_counts);
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
    double current_velocity_rpm = TalonVelocityNativeToRpm(current_velocity_native, encoder_counts);
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


//==============================================================================
// Mathematical Utilities

int KoalafiedUtilities::Modulus(int num, int div) {
    // C+ operator% usually does round toward zero division and so it not a mathematical modulus operation, which
    // needs round towards negative infinity
    if (num >= 0) return num % div;
    else          return div - 1 + (num + 1) % div;
}

double KoalafiedUtilities::Modulus(double num, double div) {
    // C+ operator% usually does round toward zero division and so it not a mathematical modulus operation, which
    // needs round towards negative infinity
    if (num >= 0) return fmod(num, div);
    else          return div - 1 + fmod((num + 1), div);
}

units::degree_t KoalafiedUtilities::NormaliseAngle(units::degree_t angle) {
    // Normalise to [0, 360)
    double angle_degrees = Modulus(angle.value(), 360.0);
    // Normalise to (-180, 180]
    if (angle_degrees > 180.0) angle_degrees -= 360.0;
    return units::degree_t(angle_degrees);
}

units::degree_t KoalafiedUtilities::NormaliseAngleDiff(units::degree_t final_angle, units::degree_t initial_angle) {
    // First normalise both angles, then compute the difference and normalise that
    final_angle = NormaliseAngle(final_angle);
    initial_angle = NormaliseAngle(initial_angle);
    return NormaliseAngle(final_angle - initial_angle);
}

units::degree_t KoalafiedUtilities::NormaliseMotorAngle(units::degree_t angle) {
    angle = units::math::abs(NormaliseAngle(angle));
    if (angle > 90_deg) angle = 180_deg - angle;
    return angle;
}


//==============================================================================
// Driver Station Utilities

bool KoalafiedUtilities::IsBlueAlliance() {
    // Get whether we are playing for the blue alliance. If no alliance is set returns true.
    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
    if (alliance) {
        return alliance.value() == frc::DriverStation::Alliance::kBlue;
    }
    return true;
}
