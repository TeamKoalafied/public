//==============================================================================
// DriveElevator.cpp
//==============================================================================

#include "DriveElevator.h"

#include "../Subsystems/Elevator.h"
#include <iostream>


//==============================================================================
// Construction

DriveElevator::DriveElevator(Elevator::LiftPosition position, bool wait_for_position):
	frc::Command("DriveElevator", 5) {

	Requires(&Elevator::GetInstance());

	m_position = position;
	m_wait_for_position = wait_for_position;

	std::cout << "DriveElevator" << (int)m_position << "\n";
}


//==============================================================================
// Function Overrides from frc::Command

void DriveElevator::Initialize() {
	std::cout << "DriveElevator::Initialize()\n";
}

void DriveElevator::Execute() {
	Elevator::GetInstance().MoveLiftToPosition(m_position);
}

bool DriveElevator::IsFinished() {
	// If we are not waiting for the lift to get to the position just return immediately
	if (!m_wait_for_position) return true;

	if (IsTimedOut()) {
		std::cout << "DriveElevator timed out\n";
		return true;
	}
	return Elevator::GetInstance().GetLiftPosition() == m_position;
}

void DriveElevator::End() {
	std::cout << "DriveElevator::End()\n";
}

void DriveElevator::Interrupted() {
    Command::Interrupted();
}
