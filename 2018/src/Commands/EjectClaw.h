//==============================================================================
// EjectClaw.h
//==============================================================================

#ifndef SRC_COMMANDS_EJECTCLAW_H_
#define SRC_COMMANDS_EJECTCLAW_H_

#include <commands/Command.h>


class EjectClaw : public frc::Command {
public:
	//==========================================================================
	// Construction
	EjectClaw();

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

};

#endif /* SRC_COMMANDS_EJECTCLAW_H_ */
