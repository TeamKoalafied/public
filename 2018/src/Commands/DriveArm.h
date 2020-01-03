//==============================================================================
// DriveArm.h
//==============================================================================

#ifndef SRC_COMMANDS_DRIVEARM_H_
#define SRC_COMMANDS_DRIVEARM_H_

#include <commands/Command.h>
#include <Timer.h>

// Command to raise or lower the elevator arm. When raising the arm the lift is first
// moved up a bit, if required, so that the cube does not catch on the robot frame.
class DriveArm : public frc::Command {
public:
    //==========================================================================
    // Construction

	// Constructor
	//
	// up - whether to move the arm up (otherwise down)
	DriveArm(bool up);

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

	bool m_arm_up;		// Whether to move the arm up (otherwise down)
	bool m_moved_lift;	// Whether this command have to move the lift
};

#endif /* SRC_COMMANDS_DRIVEARM_H_ */
