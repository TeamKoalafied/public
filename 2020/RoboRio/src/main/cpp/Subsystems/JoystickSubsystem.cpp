//==============================================================================
// JoystickSubsystem.cpp
//==============================================================================

#include "JoystickSubsystem.h"

#include "../KoalafiedUtilities.h"


//==========================================================================
// Construction

JoystickSubsystem::JoystickSubsystem(const wpi::Twine& name, int joystick_id) :
	frc::Subsystem(name) {
    m_joystick = new frc::Joystick(joystick_id);
}

JoystickSubsystem::~JoystickSubsystem() {

}


//==========================================================================
// frc::Subsystem Function Overrides

void JoystickSubsystem::InitDefaultCommand() {
    // The default command is to operate this subsystem with the joystick
    SetDefaultCommand(new JoystickCommand(this));
}

void JoystickSubsystem::Periodic() {

}


//==========================================================================
// Joystick Operation

void JoystickSubsystem::JoystickControlStarted() {
	// Clear the joystick state, otherwise press and release events that
	// have already occurred my be incorrectly reported.
	KoalafiedUtilities::ClearJoystickButtonState(m_joystick);
}

void JoystickSubsystem::JoystickControlStopped() {
    // Nothing to do
}


//==========================================================================
// class JoystickSubsystem::JoystickCommand
//==========================================================================

//==========================================================================
// Construction

JoystickSubsystem::JoystickCommand::JoystickCommand(JoystickSubsystem* joystick_subsystem) :
	  frc::Command("JoystickCommand") {
    m_joystick_subsystem = joystick_subsystem;
    Requires(joystick_subsystem);
}


//==========================================================================
// Function Overrides from frc::Command

void JoystickSubsystem::JoystickCommand::Initialize() {
    // Notify the subsystem that control with the joystick has started
    m_joystick_subsystem->JoystickControlStarted();
}

void JoystickSubsystem::JoystickCommand::Execute() {
    // Get the subsystem to control itself with the joystick
    m_joystick_subsystem->DoJoystickControl();

}

bool JoystickSubsystem::JoystickCommand::IsFinished() {

    // Operating with the joystick never finishes, but it can be interrupted
  	return false;
}

void JoystickSubsystem::JoystickCommand::End() {
    // Notify the subsystem that control with the joystick has stopped
    m_joystick_subsystem->JoystickControlStopped();
}

void JoystickSubsystem::JoystickCommand::Interrupted() {
    // Call the base class, which will just end the command
    Command::Interrupted();
}
