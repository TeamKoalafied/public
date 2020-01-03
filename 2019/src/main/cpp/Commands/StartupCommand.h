//==============================================================================
// StartupCommand.h
//==============================================================================

#ifndef StartupCommand_H
#define StartupCommand_H

#include <frc/commands/Command.h>
#include <frc/Timer.h>

using namespace frc;

 // This command runs at start up for the robot.
class StartupCommand : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	StartupCommand();

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

    frc::Timer m_timer;
    int m_execute_count;
};

#endif  // StartupCommand_H
