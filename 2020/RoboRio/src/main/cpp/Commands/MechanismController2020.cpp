//==============================================================================
// MechanismController2020.cpp
//==============================================================================

#include "MechanismController2020.h"

#include "../Subsystems/Manipulator.h"
#include <iostream>


//==========================================================================
// Overrides from IMechanismController

void MechanismController2020::DoAction(const std::string& action) {
    // Manipulator& manipulator = Manipulator::GetInstance();
	// if (action == "LiftArm") elevator.LiftArm();
	// else if (action == "DropArm") elevator.DropArm();
	// else if (action == "OpenClaw") elevator.OpenClaw();
	// else if (action == "CloseClaw") elevator.CloseClaw();
	// else if (action == "RollerGrab") elevator.RunClawRollerGrab(2);
	// else if (action == "RollerEject") elevator.RunClawRollerEject(2, false);
	// else if (action == "RollerPlace") elevator.RunClawRollerEject(1, true);
	// else if (action.rfind("MoveLift", 0) != std::string::npos) {
	// 	// The move lift has a single parameter at the end, which is the height to move the
	// 	// lift to in inches, eg "MoveLift10"
	// 	try {
	// 		double lift_height_inches = std::atof(action.substr(strlen("MoveLift")).c_str());
	// 		elevator.MoveLiftToPosition(Elevator::LiftPosition::kCustom, lift_height_inches);
	// 	}
    //     catch (std::exception & e)
    //     {
    //         std::cout << "ERROR: Error parsing mechanism action " << action << "\n";
    //         std::cout << e.what() << "\n";
    //     }		
	// }
	// else {
	// 	std::cout << "ERROR: Unrecognised mechanism command " << action << "\n";
	// }

    Manipulator& manipulator = Manipulator::GetInstance();

	if (action == "ExtendIntake") manipulator.ExtendIntake();
    else if (action == "RetractIntake") manipulator.RetractIntake();
	else if (action == "RunIndexForward") manipulator.RunIndexForward();
	else if (action == "RunIndexBack") manipulator.RunIndexBack();
	else if (action == "Shoot") manipulator.Shoot();
    else {
         std::cout << "ERROR: Unrecognised mechanism command " << action << "\n";
    }

}

