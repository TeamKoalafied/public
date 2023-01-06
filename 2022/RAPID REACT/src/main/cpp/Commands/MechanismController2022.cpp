//==============================================================================
// MechanismController2022.cpp
//==============================================================================

#include "MechanismController2022.h"

#include "../Subsystems/Manipulator.h"
#include <iostream>

//==========================================================================
// Construction

MechanismController2022::MechanismController2022() {
    m_shooting = false;
}

//==========================================================================
// Overrides from IMechanismController

void MechanismController2022::DoAction(const std::string& action) {

    Manipulator& manipulator = Manipulator::GetInstance();

    // RotateTo
    // RotateToTarget
    // Shoot




    if (action == "Shoot") {
        manipulator.StartShooter();
        m_shooting = true;
        std::cout << "Starting shooter\n";
    }
	else if (action.rfind("ShootManual", 0) != std::string::npos) {
		// The manual shooting has a single parameter at the end, which is the distance in feet to target the shooting for
		try {
			double target_distance_ft = std::atof(action.substr(strlen("ShootManual")).c_str());
			manipulator.StartShooterManual(target_distance_ft);
		}
        catch (std::exception & e)
        {
            std::cout << "ERROR: Error parsing mechanism action " << action << "\n";
            std::cout << e.what() << "\n";
        }		
	}
	else if (action == "StartIntaking") {
        manipulator.StartIntake();
    }
	else if (action == "StopShooter") {
        manipulator.StopShooter();
    }
	else if (action == "PrepareShooter") manipulator.StartPrepareShooter();
	else if (action == "None") { /* No operation */ }
    else {
         std::cout << "ERROR: Unrecognised mechanism command " << action << "\n";
    }

}

bool MechanismController2022::AreAllActionsDone() {
    if (m_shooting) {
        // Manipulator& manipulator = Manipulator::GetInstance();
        // bool shooting_complete = manipulator.BallShootCount() >= 3;
        // //std::cout << "AreAllActionsDone(): complete" << shooting_complete << "  Count: " << manipulator.BallShootCount() << "\n";
        // if (shooting_complete) {
        //     m_shooting = false;
        //     manipulator.StopShooter();
        // }
        // return shooting_complete;
    }
    return true;
}


