//==============================================================================
// TestPIDCommand.h
//==============================================================================

#ifndef TestPIDCommand_H
#define TestPIDCommand_H

#include <Commands/PIDCommand.h>


// Command for rotating given angle on the spot
class TestPIDCommand : public frc::PIDCommand {
public:
    //==========================================================================
    // Construction

    // Constructor
	//
    // turn_angle_degrees - Angle to turn through in degrees
	TestPIDCommand(double position_target);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
    //==========================================================================

protected:
    //==========================================================================
	// Function Overrides from frc::PIDCommand
	virtual double ReturnPIDInput() override;
	virtual void UsePIDOutput(double output) override;
    //==========================================================================

private:
    //==========================================================================
    // Member Variables

	double m_position_target;	//
	int m_finish_counter;

};

#endif  // DriveRotate_H
