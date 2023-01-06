//==============================================================================
// MechanismController2020.cpp
//==============================================================================

#include "MechanismController2020.h"

#include "../Subsystems/Manipulator.h"
#include <iostream>

//==========================================================================
// Construction

MechanismController2020::MechanismController2020() {
    m_shooting = false;
}

//==========================================================================
// Overrides from IMechanismController

void MechanismController2020::DoAction(const std::string& action) {

    Manipulator& manipulator = Manipulator::GetInstance();

    if (action == "Shoot") {
        manipulator.StartShooter();
        m_shooting = true;
        std::cout << "Starting shooter\n";
    }
	else if (action == "PrepareShooter") manipulator.StartPrepareShooter();
	else if (action == "StartIntaking") manipulator.StartIntaking();
	else if (action == "StopIntaking") manipulator.StopIntaking();
	else if (action == "StopIntaking") manipulator.StopIntaking();
	else if (action == "NOP") { /* No operation */ }
    else {
         std::cout << "ERROR: Unrecognised mechanism command " << action << "\n";
    }

}

bool MechanismController2020::AreAllActionsDone() {
    if (m_shooting) {
        Manipulator& manipulator = Manipulator::GetInstance();
        bool shooting_complete = manipulator.BallShootCount() >= 3;
        //std::cout << "AreAllActionsDone(): complete" << shooting_complete << "  Count: " << manipulator.BallShootCount() << "\n";
        if (shooting_complete) {
            m_shooting = false;
            manipulator.StopShooter();
        }
        return shooting_complete;
    }
    return true;
}


