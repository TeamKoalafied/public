//==============================================================================
// AutonomousCommand.h
//==============================================================================

#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <frc/commands/CommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>



 // This command runs the autonomous period for the robot.
class AutonomousCommand : public frc::CommandGroup {
public:
	// The strategy for autonomous mode
	enum class Strategy {
		kDriveForward,			// Drive forward to cross the line and do nothing else
		kLoadSwitch,			// Load the switch with the pre-loaded power cube
		kLoadScale,				// Load the scale with the pre-loaded power cube
		kTestDriveAndReturn,	// Test command to drive a distance turn around and come back
		kPathfinderSwitch,		// Load the switch with the pre-loaded power cube using Pathfinder
		kPathSwitch,			// Load the switch with the pre-loaded cube using simpler path code
		kPathScale,				// Load the scale with the pre-loaded cube using simpler path code
	};

	//The position for autonomous mode
	enum class Position {
		kLeft,		// Robot to the far left of the alliance wall
		kMiddle,	// Robot aligned against the exchange area
		kRight,		// Robot to the far right of the alliance wall
	};

    //==========================================================================
    // Construction

    // Constructor
	AutonomousCommand();

	//==========================================================================
	// Command Setup

	// Setup the command using information from the dashboard and the game data/
	// This should be called repeatedly (beginning in AutonomouseInit()) until it
	// returns true and then the this command can be started.
	//
	// Returns true if the game data was available and the command setup was complete
	bool SetupCommand();

    //==========================================================================
	// Dashboard Setup

	// Set up controls on the dashboard for choosing autonomous parameters
	static void SetupDashboard();

	// Echo the settings to the dashboard so we can check that they are working
	static void EchoSettingsToDashboard();


private:
	// The side of the field, as view from the alliance station
	enum class Side {
		kLeft,
		kRight,
	};


	//==========================================================================
	// Strategy Initialisation

	// Setup this command for the 'Drive Forward' strategy
	void InitialiseDriveForward();

	// Setup this command for the 'Load Switch' strategy
	void InitialiseLoadSwitch();

	// Setup this command for the 'Load Switch' strategy
	// void InitialiseLoadSwitchPathfinder();

	//Setup this command for the 'Load Scale' strategy
	void InitialiseLoadScale();

	// Setup this command for the 'Drive and Return' test strategy
	void InitialiseTestDriveAndReturn();

	// Setup command for the "Load Switch with Simple Path" strategy
	void InitialiseLoadSwitchPath();

	// Setup command for the "Load Scale with Simple Path" strategy
	void InitialiseLoadScalePath();


    //==========================================================================
	// Utility Functions

	// Calculate a linear interpolation between two numbers
	//
	// start - the starting value, corresponding to a ratio of 0.0
	// end - the ending value, corresponding to a ratio of 1.0
	// ratio - the ratio between the start and end values in the range [0.0, 1.0]
	double Lerp(double start, double end, double ratio);


    //==========================================================================
    // Member Variables

	Strategy m_strategy;		// The strategy for autonomous mode
	Position m_position;		// The starting position for autonomous mode
	double m_position_inch;		// The initial position of the centre of the robot in inches from the left of the field (looking out towards the field)
	std::string m_game_data;	// THe game date from the drive station that contains the switch and scale side allocations

	Side m_our_switch_side;		// The side of the switches allocated to us
	Side m_our_scale_side;		// The side of the scale allocated to us


	static frc::SendableChooser<Strategy> m_stratgey_chooser;
								// A chooser for selecting the autonomous strategy on the dashboard before the match
	static frc::SendableChooser<Position> m_position_chooser; //A chooser for selecting the autonomous position

	// Constants for the size of the robot
	static constexpr double kRobotLengthInch = 39.0;		// Length of the robot front to back in inches, including bumpers
	static constexpr double kRobotWidthInch = 34.0;			// Width of the robot side to side in inches, including bumpers

	// Constant measuring down the field (i.e. from the alliance wall outwards)
	static constexpr double kWallToAutoLineInch = 120.0;	// Alliance wall to auto line
	static constexpr double kWallToSwitchInch = 140.0;		// Alliance wall to near side of the switch in inches
	static constexpr double kWallToSwitchFarInch = 196.0;	// Alliance wall to far side of the switch in inches
	static constexpr double kWallToPlatformInch = 261.47;	// Alliance wall to near side of the platform in inches
	static constexpr double kWallToScaleInch = 299.65;		// Alliance wall to near side of the scale plate in inches

	// Constants measuring across the field (i.e. from the left to right looking down the field from the alliance station)
	static constexpr double kSideWallToAllianceInch = 29.69;// Side wall to the start of the alliance wall in inches
	static constexpr double kSideWallToSwitchInch = 85.25;  // Side wall to the near edge of the switch
	static constexpr double kSideWallToScalePlatformInch = 95.25; //Side wall to near edge of the scale platform
	static constexpr double kFieldWidthInch = 324.0;		// Full width of the field in inches

	static constexpr double kScalePlateWidthInch = 48.7; //Width of scale plate
};

#endif  // AutonomousCommand_H
