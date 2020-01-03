//==============================================================================
// DriveArm.cpp
//==============================================================================

#include "DriveArm.h"

#include "../Subsystems/Elevator.h"
#include <iostream>

//==============================================================================
// Construction

DriveArm::DriveArm(bool up):
	frc::Command("DriveArm", 2) {

	m_arm_up = up;
	m_moved_lift = false;

	// Moving the arm requires the Elevator
	Requires(&Elevator::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void DriveArm::Initialize() {
	std::cout << "DriveArm::Initialize()\n";
	m_moved_lift = false;
}

void DriveArm::Execute() {
	Elevator& elevator = Elevator::GetInstance();

	if (m_arm_up) {
		// If lifting the arm only do it if the lift is above the 'ArmLift' height,
		// otherwise drive the lift to that height
		if (elevator.GetLiftHeightInch() > elevator.GetLiftPositionHeightInch(Elevator::LiftPosition::kArmLift)) {
			elevator.LiftArm();

			// If we had to move the lift set it back to 'no position' so it gently
			// coasts back down.
			if (m_moved_lift) {
				elevator.MoveLiftToPosition(Elevator::LiftPosition::kIntermediate);
			}
		} else {
			elevator.MoveLiftToPosition(Elevator::LiftPosition::kSwitch);
			m_moved_lift = true;
		}
	} else{
		// If dropping the arm, just do it immediately
		elevator.DropArm();
	}
}

bool DriveArm::IsFinished() {
	if (IsTimedOut()){
		std::cout << "DriveArm timed out\n";
		return true;
	}

	Elevator& elevator = Elevator::GetInstance();
	if (m_arm_up) {
		// If moving the arm up we are done if it is up
		return elevator.IsArmUp();
	} else {
		// If moving the arm down we are done if it is down
		return !elevator.IsArmUp();
	}
}

void DriveArm::End() {
	std::cout << "DriveArm::End()\n";

	if (m_moved_lift) {
		Elevator& elevator = Elevator::GetInstance();
		elevator.MoveLiftToPosition(Elevator::LiftPosition::kIntermediate);
	}
}

void DriveArm::Interrupted() {
    Command::Interrupted();
}
