//==============================================================================
// DrivePath.h
//==============================================================================
//
// DrivePath works by following a simple array of { distance traveled, required speed, required heading }
// The required heading is plus or minus degrees for each leg with respect to the initial heading before the command was run
// Each line shows the speed and heading requested up to the number of inches traveled.
// The heading is pidgeon corrected, the distance traveled is purely calculated by encoder count.
//

#ifndef DrivePath_H
#define DrivePath_H

#include <Timer.h>
#include <Commands/Command.h>
#include <Commands/AutonomousCommand.h>


// Command for rotating given angle on the spot
class DrivePath : public frc::Command {
public:
	enum class PathStrategy {
		kSwitchLeft,
		kSwitchRight,
		kLeftScaleLeft,
		kLeftScaleRight,
		kRightScaleLeft,
		kRightScaleRight
	};

	enum class IntakeControl {
			kNoCmd,
	    	kIntake,
			kEject,
			kRaiseFourBarLink,
			kLowerFourBarLink,
		    kMoveElevatorToFloor,			// Lift at the lowest position
	    	kMoveElevatorToArmLift,		// Lift is high enough to lift the arm without the cube catching on the robot frame
			kMoveElevatorToSwitch,		// Lift positioned to reach the switch
			kMoveElevatorToScale,			// Lift positioned to reach the scale
			kMoveElevatorToIntermediate,	// Lift is between the preset positions, or not being driven to a position
			kStop
	    };
	struct path_array { int dist; int speed; int heading; IntakeControl IntakeCmd; };
	static const int kPathArraySize = 25;
/*
	const path_array TestDriveAndReturn_array[kPathArraySize] = {
			{2, 5, 0, IntakeControl::kMoveElevatorToFloor},
			{40, 4, 70, IntakeControl::kNoCmd},
			{50, 2, -20, IntakeControl::kMoveElevatorToSwitch},
			{60, 3, -20, IntakeControl::kLowerFourBarLink},
			{65, 2, 0, IntakeControl::kLowerFourBarLink},
			{72, 3, 0, IntakeControl::kEject},
			{73, 3, 0, IntakeControl::kMoveElevatorToFloor},
			{74, 2, 0, IntakeControl::kNoCmd},
			{75, 2, 0, IntakeControl::kStop}
	} */



	const path_array SwitchLoadLeft_array[kPathArraySize] = {
			// switch load left
					{2, 5, 0, IntakeControl::kMoveElevatorToFloor},
					{60, 5, 70, IntakeControl::kNoCmd},
					{90, 4, 30, IntakeControl::kMoveElevatorToSwitch},
					{95, 3, 0, IntakeControl::kNoCmd},
					{100, 4, -40, IntakeControl::kLowerFourBarLink},
					{109, 4, -40, IntakeControl::kNoCmd},
					{119, 3, 0, IntakeControl::kNoCmd},
					{125, 3, 0, IntakeControl::kEject},
					{126, 3, 0, IntakeControl::kNoCmd},
					{135, -5, 0, IntakeControl::kMoveElevatorToFloor},
					{136, -4, 0, IntakeControl::kNoCmd},
					{140, 2, 0, IntakeControl::kStop}
			};
	const path_array SwitchLoadRight_array[kPathArraySize] = {
			// switch load right
			{2, 5, 0, IntakeControl::kMoveElevatorToFloor},
			{60, 5, -50, IntakeControl::kNoCmd},
			{90, 4, -10, IntakeControl::kMoveElevatorToSwitch},
			{95, 3, 0, IntakeControl::kNoCmd},
			{100, 4, 40, IntakeControl::kLowerFourBarLink},
			{109, 4, 25, IntakeControl::kNoCmd},
			{117, 3, 0, IntakeControl::kNoCmd},
			{119, 3, 0, IntakeControl::kEject},
			{121, 3, 0, IntakeControl::kNoCmd},
			{135, -5, 0, IntakeControl::kMoveElevatorToFloor},
			{136, -4, 0, IntakeControl::kNoCmd},
			{140, 2, 0, IntakeControl::kStop}
	};
	const path_array LoadScaleLeftFromLeft_array[kPathArraySize] = {
			// LoadScaleLeftFromLeft_array
			{120, 8, 0 , IntakeControl::kNoCmd},
			{160, 5, 0 , IntakeControl::kNoCmd},
			{190, 5, -90, IntakeControl::kMoveElevatorToScale},
			{220, 5, -50, IntakeControl::kNoCmd},
			{221, 5, 55, IntakeControl::kNoCmd}, //75
			{235, 5, 55, IntakeControl::kNoCmd},
			{279, 5, 0,  IntakeControl::kNoCmd},
			{281, 5, 0, IntakeControl::kEject},
			{320, -6, -90, IntakeControl::kNoCmd},
			{350, 5, -180, IntakeControl::kMoveElevatorToFloor},
			{355, 5, -180, IntakeControl::kNoCmd},
			{360, 5, -180, IntakeControl::kStop}
	};

	const path_array LoadScaleRightFromLeft_array[kPathArraySize] = {
			{130, 8, 0 , IntakeControl::kNoCmd},
			{165, 5, 0, IntakeControl::kNoCmd},
			{335, 5, -90, IntakeControl::kNoCmd},
			{337, 5, -90, IntakeControl::kMoveElevatorToScale},
			{341, 5, -90, IntakeControl::kNoCmd},
			{360, 5, -90, IntakeControl::kNoCmd},
			{365, 4, 10, IntakeControl::kNoCmd},
			{436, 3, 0, IntakeControl::kNoCmd},
			{438, 3, 0, IntakeControl::kEject},
			{440, -5, 0, IntakeControl::kNoCmd},
			{445, -5, 40, IntakeControl::kMoveElevatorToFloor},
			{450, 5, 170, IntakeControl::kNoCmd},
			{455, 5, 170, IntakeControl::kStop},
			{460, 5, 170, IntakeControl::kStop}
	};


	const path_array LoadScaleRightFromRight_array[kPathArraySize] = {
			// LoadScaleRightFromRight_array
			{120, 8, 0 , IntakeControl::kNoCmd},
			{160, 5, 0 , IntakeControl::kNoCmd},
			{190, 5, 90, IntakeControl::kMoveElevatorToScale},
			{220, 5, 50, IntakeControl::kNoCmd},
			{221, 5, -55, IntakeControl::kNoCmd}, //-75
			{235, 5, -55, IntakeControl::kNoCmd},
			{279, 5, 0,  IntakeControl::kNoCmd},
			{281, 5, 0, IntakeControl::kEject},
			{320, -6, 90, IntakeControl::kNoCmd},
			{350, 5, 180, IntakeControl::kMoveElevatorToFloor},
			{355, 5, 180, IntakeControl::kNoCmd},
			{360, 5, 180, IntakeControl::kStop}
	};
/*	const path_array LoadScaleLeftFromRight_array[kPathArraySize] = {
			{130, 8, 0 , IntakeControl::kNoCmd},
			{140, 5, 0, IntakeControl::kNoCmd},
			{335, 5, 90, IntakeControl::kNoCmd},
			{336, 5, 90, IntakeControl::kNoCmd},
			{338, 6, 90, IntakeControl::kMoveElevatorToScale},
			{339, 6, 90, IntakeControl::kNoCmd},
			{340, 5, -20, IntakeControl::kNoCmd},
			{441, 5, 0, IntakeControl::kNoCmd},
			{442, 5, 0, IntakeControl::kEject},
			{465, -5, 0, IntakeControl::kNoCmd},
			{470, -5, -40, IntakeControl::kMoveElevatorToFloor},
			{490, 5, -170, IntakeControl::kNoCmd},
			{500, 5, -170, IntakeControl::kStop}
	}; */

	const path_array LoadScaleLeftFromRight_array[kPathArraySize] = {
			{130, 8, 0 , IntakeControl::kNoCmd},
			{165, 5, 0, IntakeControl::kNoCmd},
			{335, 5, 90, IntakeControl::kNoCmd},
			{337, 5, 90, IntakeControl::kMoveElevatorToScale},
			{341, 5, 90, IntakeControl::kNoCmd},
			{360, 5, 90, IntakeControl::kNoCmd},
			{365, 4, -10, IntakeControl::kNoCmd},
			{436, 3, 0, IntakeControl::kNoCmd},
			{438, 3, 0, IntakeControl::kEject},
			{440, -5, 0, IntakeControl::kNoCmd},
			{445, -5, -40, IntakeControl::kMoveElevatorToFloor},
			{450, 5, -170, IntakeControl::kNoCmd},
			{455, 5, -170, IntakeControl::kStop},
			{460, 5, -170, IntakeControl::kStop}
	};

	const path_array TestDriveAndReturn_array[kPathArraySize] = {
			// LoadScaleLeftFromLeft_array
			{48, 5, 0 , IntakeControl::kNoCmd},
			{50, 5, 0, IntakeControl::kStop}
	};

    //==========================================================================
    // Construction

    // Constructor
	//
    // turn_angle_degrees - Angle to turn through in degrees
	DrivePath(PathStrategy Strategy);

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
	int m_target_distance;
	int m_target_speed;
	int m_distance_travelled;
	int m_start_point_inch;
	int m_initial_heading_degrees;
	path_array const *m_lookup;
	int mStop;
	int m_last_reading;

	double m_target_heading_degrees;	// Heading angle to turn to for the current execution in degrees
	int m_finish_counter;				// Counter for detecting if the command is complete
	int state;							// State for controlling rotation
	frc::Timer m_timer;					// Timer for measuring total command duration
};

#endif  // DrivePath_H
