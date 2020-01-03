//==============================================================================
// DriveRotatePID.cpp
//==============================================================================

#include "DriveRotatePID.h"

#include "../Subsystems/DriveBase.h"

#include <SmartDashboard/SmartDashboard.h>

#include <iostream>
namespace {
	const double kP = 0.003;
	const double kI = 0.0;
	const double kD = 0.0;
	const double kPeriod = 0.02; // 20ms as normal for period
}

//==============================================================================
// Construction

DriveRotatePID::DriveRotatePID(AngleType angle_type, double turn_angle_degrees) :
	frc::Command("DriveRotatePID") {

	m_angle_type = angle_type;
    m_turn_angle_degrees = turn_angle_degrees;

    std::cout << "DriveRotatePID Angle " << m_turn_angle_degrees << " Type " << (int)m_angle_type << "\n";


    // Set the timeout for this command to something a bit longer than a full 360
    // degree turn would take
    // TODO: Hack to 5s for now
    const double kTimeoutS = 5.0;
    SetTimeout(kTimeoutS);

	m_pid_controller = new frc::PIDController(0.01, 0.0, 0.0, this, this, 0.02);
//	m_pid_controller->SetInputRange(-10.0, 10.0);
	m_pid_controller->SetOutputRange(-1.0, 1.0);
	m_pid_controller->SetAbsoluteTolerance(2.0);
//	m_pid_controller->SetToleranceBuffer(10);

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void DriveRotatePID::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "=============================================================================\n";
	std::cout << "DriveRotatePID::Initialize()\n";


    // Calculate the heading to turn to
    double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    switch (m_angle_type) {
    	default:
    	case AngleType::kAbsolute:
    	    m_target_heading_degrees = m_turn_angle_degrees;
    		break;
    	case AngleType::kRelative:
    	    m_target_heading_degrees = current_heading + m_turn_angle_degrees;
    		break;
    }
    std::cout << "Current Pigeon Heading: " << current_heading << "\n";
    std::cout << "Target Heading: " << m_target_heading_degrees << "\n";


    m_pid_controller->SetSetpoint(m_target_heading_degrees);
    m_pid_controller->Enable();
    m_finish_counter = 0;
}

void DriveRotatePID::Execute() {
//	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
//    double error = m_target_heading_degrees - current_heading;
//    SmartDashboard::PutNumber("HError", error);
}

bool DriveRotatePID::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

    if (IsTimedOut()) {
        printf("DriveRotatePID command timed out\n");
        return true;
    }
    if (m_pid_controller->OnTarget()) {
    	m_finish_counter++;
    } else {
    	m_finish_counter = 0;
    }

    return m_finish_counter > 10;
}

void DriveRotatePID::End() {
	// Stop the PID controller
    m_pid_controller->Disable();

    // Ensure the drive base is stopped
    DriveBase::GetInstance().Stop();

    std::cout << "DriveRotatePID::End()\n";
}

//==========================================================================
// Function Overrides from frc::PIDSource

double DriveRotatePID::PIDGet()
{
	return DriveBase::GetInstance().GetPigeonHeading();
}

//==========================================================================
// Function Overrides from frc::PIDOutput

void DriveRotatePID::PIDWrite(double output)
{
    DriveBase::GetInstance().Drive(0.0, output);
}



//==========================================================================
// Function Overrides from frc::CPIDCommandommand
//
//double DriveRotatePID::ReturnPIDInput() {
//	return DriveBase::GetInstance().GetPigeonHeading();
//}
//
//
//void DriveRotatePID::UsePIDOutput(double output) {
//	double rotate = output;
//
//    // Clip the rotate to the allowed range [-1, 1] and update the drive base
//    if (rotate > 1.0) rotate = 1.0;
//    if (rotate < -1.0) rotate = -1.0;
//
//    // 0.09 for alan, 0.05 for alan2
//    if (rotate > 0) {
//        if (rotate < 0.04) rotate = 0.04;
//    } else {
//       if (rotate > -0.04) rotate = -0.04;
//    }
//
//    DriveBase::GetInstance().Drive(0.0, rotate);
//}

