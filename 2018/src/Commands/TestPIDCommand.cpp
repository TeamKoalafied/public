//==============================================================================
// TestPIDCommand.cpp
//==============================================================================

#include "TestPIDCommand.h"

#include "../Subsystems/TestSubsystem.h"

#include <SmartDashboard/SmartDashboard.h>

#include <iostream>
namespace {
	const double kP = 0.003;
	const double kI = 0.0;
	const double kD = 0.0;
	const double kPeriod = 0.05; // 20ms as normal for period
}

//==============================================================================
// Construction

TestPIDCommand::TestPIDCommand(double position_target) :
	frc::PIDCommand("TestPIDCommand", kP, kI, kD, kPeriod) {
    m_position_target = position_target;

    // Set the timeout for this command to something a bit longer than a full 360
    // degree turn would take
    // TODO: Hack to 5s for now
    const double kTimeoutS = 15.0;
    SetTimeout(kTimeoutS);

	// Driving requires the DriveBase
    Requires(&TestSubsystem::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void TestPIDCommand::Initialize() {
    // Called just before this Command runs the first time

	printf("=============================================================================\n");
    printf("TestPIDCommand::Initialize()\n");

    TestSubsystem::GetInstance().ResetDriving();

    SetSetpoint(m_position_target);

    m_finish_counter = 0;

    std::cout << "Setpoint set to " << m_position_target << "\n";

    frc::PIDCommand::Initialize();
}

void TestPIDCommand::Execute() {
//	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
//    double error = m_target_heading_degrees - current_heading;
//    SmartDashboard::PutNumber("HError", error);
//	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
//    double error = m_target_heading_degrees - current_heading;
//    SmartDashboard::PutNumber("HError", error);
    frc::PIDCommand::Execute();
}

bool TestPIDCommand::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

    if (IsTimedOut()) {
        printf("TestPIDCommand command timed out\n");
        return true;
    }
    return false;

//	// Get the current heading and calculate an error to our desired heading
//	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
//    double error = m_target_heading_degrees - current_heading;
//
//    // The command is done if the error is below a threshold margin
//    const double kRotateMargin = 2.0;
//    if (fabs(error) < kRotateMargin) {
//        m_finish_counter++;
//    }else {
//        m_finish_counter = 0;
//    }
//
//    return m_finish_counter >= 20;
}

//==========================================================================
// Function Overrides from frc::CPIDCommandommand

double TestPIDCommand::ReturnPIDInput() {
	std::cout << "TestPIDCommand::ReturnPIDInput()\n";
	return 10.0;
//	return TestSubsystem::GetInstance().GetPosition();
}


void TestPIDCommand::UsePIDOutput(double output) {
	std::cout << "TestPIDCommand::UsePIDOutput(" << output << ")\n";

	double rotate = output;

    // Clip the rotate to the allowed range [-1, 1] and update the drive base
    if (rotate > 1.0) rotate = 1.0;
    if (rotate < -1.0) rotate = -1.0;

    // 0.09 for alan, 0.05 for alan2
    if (rotate > 0) {
        if (rotate < 0.04) rotate = 0.04;
    } else {
       if (rotate > -0.04) rotate = -0.04;
    }

//    TestSubsystem::GetInstance().Drive(rotate);
}
