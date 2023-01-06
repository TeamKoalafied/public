//==============================================================================
// VisionFindTarget.h
//==============================================================================

#ifndef VisionFindTarget_H
#define VisionFindTarget_H

#include <frc/commands/Command.h>

// This command uses the vision system to find a target and drives up to it
class VisionFindTarget : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	VisionFindTarget();

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================
	// Utility functions
	void TurnOnFloodlight();
	void TurnOffFloodlight();

private:
    enum mode_t {
        // Entry states for each test
		kvision = 0,			// Use vision feedback with openloop motor control

		// Intermediate states
		kvision_check,
		kfind_target,
		kgrab_target,
		kdone,
		ktimer
    };

    //==========================================================================
	// Member Variables

    mode_t m_mode;          // Current mode
    mode_t m_next_mode;     // Next mode when timed out
    int m_timer;            // countdown timer to use when mode is ktimer
	double m_target_bearing;

    //==========================================================================
    // Utility functions

    double GetSpeed(double distance, double angle);
    double GetRotation(double distance, double angle);
};

#endif  // VisionFindTarget_H
