//==============================================================================
// TestCharacteriseDriveBase.h
//==============================================================================


#include <frc/commands/Command.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <vector>



#ifndef SRC_COMMANDS_TESTCHARACTERISEDRIVEBASE_H_
#define SRC_COMMANDS_TESTCHARACTERISEDRIVEBASE_H_

// Command for performing drive base characterisation tests
class TestCharacteriseDriveBase : public frc::Command {
public:
    //==========================================================================
	// Nested Types

	// The type of tests that can be performed
	enum Type {
		SteadyStateVoltage,				//
		Acceleration,
		QuasiStaticVoltage,
		MinType = SteadyStateVoltage,	// Minimum UI settings
		MaxType = QuasiStaticVoltage,	// Minimum UI settings
	};

    //==========================================================================
    // Construction

    // Constructor
    //
    // type -
	TestCharacteriseDriveBase(Type type, double voltage, double maximum_distance_feet);
	virtual ~TestCharacteriseDriveBase();


    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================


    //==========================================================================
	// Static Joystick Control Functions

	// Do joystick control of drivebase characterisation
	//
	// joystick - The joystick to query for user input
	static void DoJoystickControl(frc::Joystick* joystick);

private:
    //==========================================================================
	// Nested Types

	// Sample data recorded during a test
	struct Sample
	{
		double m_time_s;		// Time in seconds the sample was taken at relative to the start of the test
		double m_voltage;		// Current voltage to the motor as measured by the talon
		double m_velocity_fps;	// Current velocity in feet per second
	};

	// Possible settings that can set in the UI
	enum UISetting
	{
		TypeSetting,		// UI is setting the type of test to perform
		VoltageSetting,		// UI is setting the voltage to test at
		MaxDistanceSetting,	// UI is setting the maximum distance to move
		MinUISetting = TypeSetting,			// Minimum UI settings
		MaxUISetting = MaxDistanceSetting,	// Minimum UI settings
	};


    //==========================================================================
	// Results

	// Write the results of the test to the output file
	void WriteResultsToFile();


    //==========================================================================
	// Static Joystick Control Implementation Functions

	// Set the current UI setting being manipulated, and display the new values
	//
	// ui_setting - The UI setting to set. Will be clipped to the valid range.
	static void SetUiSetting(UISetting ui_setting);

	// Set the type of test to perform, and display the new values
	//
	// type_setting - The type of test to perform to set. Will be clipped to the valid range.
	static void SetTypeSetting(Type type_setting);

	// Set the voltage, and display the new values
	//
	// voltage - The voltage to set. Will be clipped to the valid range.
	static void SetVoltageSetting(double voltage);

	// Set the maximum distance to go during a test in feet, and display the new values
	//
	// distance_feet - Theaximum distance to go during a test in feet to set. Will be clipped to the valid range.
	static void SetMaxDistanceFeetSetting(double distance_feet);

	// Display the current configuration set in the UI
	static void DisplayConfigInLog();


    //==========================================================================
	// Member Variables

	Type m_type;            					// Type of test to perform
	double m_voltage;				  			// Voltage to apply for the test
	double m_maximum_distance_feet;				// Maximum distance to go during a test in feet
    frc::Timer m_timer;         				// A timer that measures elapsed time
    std::vector<Sample> m_sample_list;			// List of data samples recorded during the test


    static UISetting ms_ui_setting;				// What value the UI is currently changing
    static bool ms_pov_down;					// Whether the POV control is currently down on any value
    static Type ms_type_setting;	    		// Type of test as set by the user
    static double ms_voltage_setting;	    	// Voltage for the test set by the user
    static double ms_max_distance_feet_setting;	// Maximum distance to go during a test in feet set by the user

    static TestCharacteriseDriveBase* ms_command;
};

#endif /* SRC_COMMANDS_TESTCHARACTERISEDRIVEBASE_H_ */
