//==============================================================================
// VisionFindCube.h
//==============================================================================

#ifndef VisionFindCube_H
#define VisionFindCube_H

#include <frc/Relay.h>
#include <frc/commands/Command.h>



// This command uses the vision system to find a cube and drives up to it
class VisionFindCube : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	VisionFindCube();

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
        kfind_cube,
        kgrab_cube,
        kdone,
		killuminate_scene
    };

    //==========================================================================
	// Member Variables

    mode_t m_mode;          // Current mode
    double m_old_direction_of_object;
    int m_test_counter;
    frc::Relay m_relay{0};

    //==========================================================================
    // Utility functions

    double GetSpeed(double distance, double angle);
    double GetRotation(double distance, double angle);
};

#endif  // VisionFindCube_H
