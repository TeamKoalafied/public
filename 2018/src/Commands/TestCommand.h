//==============================================================================
// TestCommand.h
//==============================================================================

#ifndef TestCommand_H
#define TestCommand_H

#include "../PeriodicTimer.h"

#include <Timer.h>
#include <PIDController.h>
#include <PIDOutput.h>
#include <PIDSource.h>
#include <Commands/Command.h>


// This command flashes one of the LEDs on for a given period of time
class TestCommand : public frc::Command, private frc::PIDSource, private frc::PIDOutput {
public:
	enum Type {
		DoJoystick,
		FlashLedForever,
		PID
	};

    //==========================================================================
    // Construction

    // Constructor
    //
    // type -
	TestCommand(Type type);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

private:
    //==========================================================================
	// Function Overrides from frc::PIDSource
	virtual double PIDGet() override;
    //==========================================================================

    //==========================================================================
	// Function Overrides from frc::PIDOutput
	virtual void PIDWrite(double output) override;
    //==========================================================================

    //==========================================================================
	// Member Variables

	Type m_type;            		// Command to perform
    frc::Timer m_timer;         	// A timer that measures elapsed time
    frc::PIDController* m_pid_controller;
	PeriodicTimer m_periodic_timer;         // Timer for monitoring response times
};

#endif  // FlashLed_H
