//==============================================================================
// JoystickSubsystem.h
//==============================================================================

#pragma once

#include <frc/commands/Command.h>
#include <frc/commands/Subsystem.h>

namespace frc
{
class Joystick;
}


class JoystickSubsystem : public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    JoystickSubsystem(const wpi::Twine& name, int joystick_id);

	  // Destructor
	  virtual ~JoystickSubsystem();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Joystick Operation

    // Setup joystick control of this subsystem. Called each time joystick control starts.
    virtual void JoystickControlStarted();

    // Do joystick control of this subsystem
    virtual void DoJoystickControl() = 0;

    // Tidy up joystick control of this subsystem. Called each time joystick control stops.
    virtual void JoystickControlStopped();

protected:
  	// Get the joystick used to control this subsystem
	  frc::Joystick* GetJoystick() { return m_joystick; }

private:

    // Command to operate the elevator with the joystick. This is the default command
    // for the elevator. This command run forever, until interupted, and simply tells
    // the Elevator to do joystick operation.
    class JoystickCommand : public frc::Command {
    public:
        //==========================================================================
        // Construction

        // Constructor
        JoystickCommand(JoystickSubsystem* joystick_subsystem);

        //==========================================================================
        // Function Overrides from frc::Command
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End() override;
        void Interrupted() override;
        //==========================================================================

    private:
        JoystickSubsystem* m_joystick_subsystem;
    };
    

    //==========================================================================
    // Member Variables

	  frc::Joystick* m_joystick;			 	// The joystick, used to control this subsystem
};
