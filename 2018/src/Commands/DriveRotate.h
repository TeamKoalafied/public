//==============================================================================
// DriveRotate.h
//==============================================================================

#ifndef DriveRotate_H
#define DriveRotate_H

#include <Timer.h>
#include <Commands/Command.h>


// Command for rotating given angle on the spot
class DriveRotate : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	//
    // turn_angle_degrees - Angle to turn through in degrees
	DriveRotate(double turn_angle_degrees);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

	// Set the turn angle
	//
    // turn_angle_degrees - Angle to turn through in degrees
	void SetTurnAngle(double turn_angle_degrees) { m_turn_angle_degrees = turn_angle_degrees; }


private:
    //==========================================================================
    // Member Variables

	double m_turn_angle_degrees;		// Angle to turn through in degrees

	double m_target_heading_degrees;	// Heading angle to turn to for the current execution in degrees
	int m_finish_counter;				// Counter for detecting if the command is complete
	int state;							// State for controlling rotation
	frc::Timer m_timer;					// Timer for measuring total command duration
};

#endif  // DriveRotate_H
