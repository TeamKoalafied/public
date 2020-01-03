//==============================================================================
// EjectClaw.cpp
//==============================================================================

#include "EjectClaw.h"

#include "../Subsystems/Elevator.h"

//==========================================================================
// Construction

EjectClaw::EjectClaw():
	frc::Command("EjectClaw", 2) {

	Requires(&Elevator::GetInstance());
}


//==========================================================================
// Function Overrides from frc::Command

void EjectClaw::Initialize() {
}

void EjectClaw::Execute() {
	Elevator::GetInstance().RunClawRollerEject(1,false);
}

bool EjectClaw::IsFinished() {
	return IsTimedOut();
}

void EjectClaw::End() {
}

void EjectClaw::Interrupted() {
    Command::Interrupted();
}
