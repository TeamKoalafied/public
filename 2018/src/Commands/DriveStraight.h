//==============================================================================
// DriveStraight.h
//==============================================================================

#ifndef DriveStraight_H
#define DriveStraight_H

#include <Timer.h>
#include <Commands/Command.h>


// Command for driving autonomously for a given distance in a straight line
class DriveStraight : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
	//
    // velocity_feet_per_second - Velocity to drive at in feet/s
    // distance_m - Distance to drive in inches
	// heading - Heading to drive straight along
	DriveStraight(double velocity_feet_per_second, double distance_inch, double heading);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

	// Set the command parameters
	//
    // velocity_feet_per_second - Velocity to drive at in feet/s
    // distance_m - Distance to drive in inches
	// heading - Heading to drive straight along
	void Set(double velocity_feet_per_second, double distance_inch, double heading);

private:
    //==========================================================================
	// Timeout Calculation

	// Calculate and set a timeout for the current parameters
	void SetupTimeout();


    //==========================================================================
    // Member Variables

	double m_velocity_feet_per_second;	// Velocity to drive at in feet/s
	double m_distance_inch;        		// Distance to drive in inches
	double m_heading;					// Heading to drive straight along

	double m_target_distance_inch;      // The encode distance to drive to in inches

	bool m_braking;						// The braking phase has begun
	double m_braking_distance;			// The distance from the final target position to begin breaking at
	frc::Timer m_timer;					// Timer for measuring total command duration
};

#endif  // DriveStraight_H
