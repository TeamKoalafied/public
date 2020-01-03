//==============================================================================
// DriveElevator.h
//==============================================================================

#ifndef DriveElevator_H
#define DriveElevator_H

#include <Commands/Command.h>
#include <Timer.h>
#include "../Subsystems/Elevator.h"

// Command to drive the elevator lift to a given position. It can wait for the
// movement to complete or end immediately.
class DriveElevator : public frc::Command {
public:
    //==========================================================================
    // Construction

	// Constructor
	//
	// position - The predefined lift position to move to
	// wait_for_position - When the command waits for the lift to get to the position, or ends immediately
	DriveElevator(Elevator::LiftPosition position, bool wait_for_position);

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
    // Member Variables

	Elevator::LiftPosition m_position;		// The position to drive the elevator lift to
	bool m_wait_for_position;				// Whether to wait for the lift to get to the position
};

#endif /* DriveElevator_H */
