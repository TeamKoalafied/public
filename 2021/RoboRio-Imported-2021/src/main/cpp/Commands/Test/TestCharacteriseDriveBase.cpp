//==============================================================================
// TestCharacteriseDriveBase.cpp
//==============================================================================


#include "TestCharacteriseDriveBase.h"
#include "../../Subsystems/DriveBase.h"
#include "../../RobotConfiguration.h"
#include <fstream>
#include <iostream>
#include <iomanip>

#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Timeout if the command ever takes more than 20s
#define TIMEOUT_S 20

//==============================================================================

TestCharacteriseDriveBase::UISetting TestCharacteriseDriveBase::ms_ui_setting = TestCharacteriseDriveBase::TypeSetting;
bool TestCharacteriseDriveBase::ms_pov_down = false;
TestCharacteriseDriveBase::Type TestCharacteriseDriveBase::ms_type_setting = TestCharacteriseDriveBase::SteadyStateVoltage;
double TestCharacteriseDriveBase::ms_voltage_setting = 3.0;
double TestCharacteriseDriveBase::ms_test_duration_setting = 6.0;
TestCharacteriseDriveBase* TestCharacteriseDriveBase::ms_command = NULL;

//==============================================================================

TestCharacteriseDriveBase::TestCharacteriseDriveBase(Type type, double voltage, double test_duration) :
	frc::Command("TestCommand", type != PigeonDrift ? TIMEOUT_S : test_duration * 10) {

	m_type = type;
	m_voltage = voltage;
	m_test_duration = test_duration;

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());

}

TestCharacteriseDriveBase::~TestCharacteriseDriveBase() {
}

//==============================================================================
// Function Overrides from frc::Command

void TestCharacteriseDriveBase::Initialize() {
	// Reset the drive base distance measurement
	DriveBase& drive_base = DriveBase::GetInstance();
	drive_base.ResetDistance();
	drive_base.ResetPigeonHeading();
	drive_base.ResetPosition();
//	m_initial_heading = drive_base.GetPigeonHeading();
	m_initial_heading = 0.0;

	// Drive the robot with the set open loop voltage according to the type of test
	double output = m_voltage / 12.0;
	switch (m_type) {
		case SteadyStateVoltage:
			// Drive left and right forward to drive straight ahead
		    drive_base.TankDriveOpenLoop(output, output);
			break;
		case SteadyStateVoltageRotation:
			// Drive left and right in opposite directions so that the robot turns on the spot (in the direction of increading heading)
			drive_base.TankDriveOpenLoop(-output, output);
			break;
		case PigeonDrift:
			// Set the drive to zero and we want to test the pigeon drift without any movement
		    drive_base.TankDriveOpenLoop(0.0, 0.0);
			break;
	}

	// Clear the list of samples
	m_sample_list.clear();
	int maximum_samples = (TIMEOUT_S /0.02) + 10;
	m_sample_list.reserve(maximum_samples);

	// Restart the timer
    m_timer.Reset();
    m_timer.Start();
}

void TestCharacteriseDriveBase::Execute() {
	// Record a data sample
	DriveBase& drive_base = DriveBase::GetInstance();
	Sample sample;
	sample.m_time_s = m_timer.Get();
	sample.m_voltage = drive_base.GetMotorVoltage();
	if (m_type == SteadyStateVoltageRotation) sample.m_voltage = -sample.m_voltage;
	sample.m_velocity_fps = drive_base.GetVelocityFeetPerSecond();
	sample.m_heading = drive_base.GetPigeonHeading();
	sample.m_current = drive_base.GetMotorCurrent();

	// If this is the first sample use the initial heading because often the pigeon does not reset
	if (m_sample_list.empty()) sample.m_heading = m_initial_heading;

	m_sample_list.push_back(sample);
}

bool TestCharacteriseDriveBase::IsFinished() {

	DriveBase& drive_base = DriveBase::GetInstance();
	double current_distance_inch = drive_base.GetDistanceInch();
	double current_heading = drive_base.GetPigeonHeading();
	frc::SmartDashboard::PutNumber("CharacteriseDistance", current_distance_inch);

	bool exceeded_max_distance = false;
	switch (m_type) {
		case SteadyStateVoltage:
			// Test duration is a distance in feet
			exceeded_max_distance = (current_distance_inch > m_test_duration * 12.0);
			break;
		case SteadyStateVoltageRotation:
			// Test duration is a number of rotations
			exceeded_max_distance = (current_heading - m_initial_heading)/90.0 > m_test_duration;
			break;
		case PigeonDrift:
			// Nothing to do. The timeout controls the length of the test.
			break;
	}

	return exceeded_max_distance || IsTimedOut();
}

void TestCharacteriseDriveBase::End() {
	// Stop the robot
	DriveBase& drive_base = DriveBase::GetInstance();
	drive_base.Stop();

	// Write the results to a file
	WriteResultsToFile();

	// Display a sample of the results in a simple form in the log (display every 10th sample)
	int i = 0;
	int total_velocities = m_sample_list.size();
	while (i < total_velocities) {
		int limit = i + 50;
		if (limit > total_velocities) limit = total_velocities;
		while (i < limit) {
			std::cout << " [" << std::setprecision(3) << m_sample_list[i].m_time_s << " "  << m_sample_list[i].m_velocity_fps << " "
				      << m_sample_list[i].m_voltage << " "  << m_sample_list[i].m_heading << "]";
			i += 10;
		}
		std::cout << "\n";
	}
}

void TestCharacteriseDriveBase::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
	std::cout << "TestCharacteriseDriveBase::Interrupted()\n";

    // Call the base class, which will just end the command
    Command::Interrupted();
}


//==========================================================================
// Results

// Write the results of the test to the output file
void TestCharacteriseDriveBase::WriteResultsToFile()
{
	char cwd[PATH_MAX];
	if (getcwd(cwd, sizeof(cwd)) != NULL) {
		printf("Current working dir: %s\n", cwd);
	} else {
		printf("getcwd() error");
	}

	const char* const RESULT_FILENAME = "/home/lvuser/TestCharacteriseDriveBase.csv";
	std::ofstream results_file;
	results_file.open(RESULT_FILENAME, std::ios::out | std::ios::app);
	if (results_file.fail()) {
		std::cout << "TestCharacteriseDriveBase::WriteResultsToFile() - Failed to open result file\n";
		return;
	}

	// Write the type of test
	results_file << "\"Test Type\",";
	switch (m_type) {
		case SteadyStateVoltage:         results_file << "\"SteadyStateVoltage\"\n"; break;
		case SteadyStateVoltageRotation: results_file << "\"SteadyStateVoltageRotation\"\n"; break;
		case PigeonDrift:                results_file << "\"PigeonDrift\"\n"; break;
	}

	// Write the nominal test veloicty
	results_file << "\"Test Voltage\"," << m_voltage << "\n";

	// Write the sample times in a single line. Use 3 digits of fixed precision as the time acculumates to large
	//values but we are still interesting in small differences (at least milliseconds).
	results_file << "\"Time (s)\"";
	int total_samples = m_sample_list.size();
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_time_s;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the sample voltages in a single line
	results_file << "\"Voltage (V)\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_voltage;
	results_file << "\n";

	// Write the sample velocities in a single line
	results_file << "\"Velocity (fps)\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_velocity_fps;
	results_file << "\n";

	// Write the sample headings in a single line. Use 2 digits of fixed precision as the heading acculumates to 3
	// or 4 digit values but we are still interesting in small differences.
	results_file << "\"Heading (degrees)\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(2) << m_sample_list[i].m_heading;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the sample currents in a single line
	results_file << "\"Current (A)\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_current;
	results_file << "\n";

	// Blank line to allow automatic test analysis to tell where the test ends
	results_file << "\n";
}


//==========================================================================
// Static Joystick Control Functions

void TestCharacteriseDriveBase::DoJoystickControl(frc::Joystick* joystick)
{
	// We do the joystick control of characterisation here so that only a single line
	// of code calling this function need to be added to the DriveBase when we want to
	// characterise the drivebase.

	const double VOLTAGE_STEP = 1.0;
	const double DURATION_STEP = 1.0;

	int pov_angle = joystick->GetPOV(0);
	if (pov_angle == -1) {
		// The POV is currently not pressed. Clear the down flag.
		ms_pov_down = false;
	} else if (!ms_pov_down) {
		// The POV has just been pressed down, so execute the appropriate change.
		ms_pov_down = true;

		switch (pov_angle) {
			case 0:
				// POV Up - Increase the currently selected UI setting
				switch (ms_ui_setting) {
					case TypeSetting: 	  	 SetTypeSetting((Type)(ms_type_setting + 1)); break;
					case VoltageSetting:     SetVoltageSetting(ms_voltage_setting + VOLTAGE_STEP); break;
					case MaxDistanceSetting: SetTestDurationSetting(ms_test_duration_setting + DURATION_STEP); break;
				}
				break;
			case 180:
				// POV Down - Decrease the currently selected UI setting
				switch (ms_ui_setting) {
					case TypeSetting: 	  	 SetTypeSetting((Type)(ms_type_setting - 1)); break;
					case VoltageSetting:     SetVoltageSetting(ms_voltage_setting - VOLTAGE_STEP); break;
					case MaxDistanceSetting: SetTestDurationSetting(ms_test_duration_setting - DURATION_STEP); break;
				}
				break;
			case  90:
				// POV Right - move to editing the next UI setting
				SetUiSetting((UISetting)(ms_ui_setting + 1));
				break;
			case 270:
				// POV Left - move to editing the previous UI setting
				SetUiSetting((UISetting)(ms_ui_setting - 1));
				break;
		}
	}


    if (joystick->GetRawButtonPressed(RobotConfiguration::kJoystickAButton)) {
    	delete ms_command;
    	ms_command = new TestCharacteriseDriveBase(ms_type_setting, ms_voltage_setting, ms_test_duration_setting);
     	ms_command->Start();

// Fake data for testing writing to a file
//    	for (int i = 0; i < 10; i++) {
//    		Sample sample;
//    		sample.m_time_s = i;
//    		sample.m_voltage = i;
//    		sample.m_velocity_fps = i;
//    		ms_command->m_sample_list.push_back(sample);
//    	}
//    	ms_command->WriteResultsToFile();

    }
}

void TestCharacteriseDriveBase::SetUiSetting(UISetting ui_setting)
{
	if (ui_setting < MinUISetting) ui_setting = MinUISetting;
	if (ui_setting > MaxUISetting) ui_setting = MaxUISetting;
	ms_ui_setting = ui_setting;
	DisplayConfigInLog();
}

void TestCharacteriseDriveBase::SetTypeSetting(Type type_setting)
{
	if (type_setting < MinType) type_setting = MinType;
	if (type_setting > MaxType) type_setting = MaxType;
	ms_type_setting = type_setting;
	DisplayConfigInLog();
}

void TestCharacteriseDriveBase::SetVoltageSetting(double voltage)
{
	const double MIN_VOLTAGE = 1.0;
	const double MAX_VOLTAGE = 12.0;
	if (voltage < MIN_VOLTAGE) voltage = MIN_VOLTAGE;
	if (voltage > MAX_VOLTAGE) voltage = MAX_VOLTAGE;
	ms_voltage_setting = voltage;
	DisplayConfigInLog();
}

void TestCharacteriseDriveBase::SetTestDurationSetting(double test_duration)
{
	const double MIN_DURATION = 2.0;
	const double MAX_DURATION = 30.0;
	if (test_duration < MIN_DURATION) test_duration = MIN_DURATION;
	if (test_duration > MAX_DURATION) test_duration = MAX_DURATION;
	ms_test_duration_setting = test_duration;
	DisplayConfigInLog();
}

void TestCharacteriseDriveBase::DisplayConfigInLog()
{
	std::cout << "TestCharacteriseDriveBase Config - ";
	std::cout << (ms_ui_setting == TypeSetting ? "*Type*: " : " Type : ");
	switch (ms_type_setting) {
		case SteadyStateVoltage:          std::cout << "SteadyStateVoltage        "; break;
		case SteadyStateVoltageRotation:  std::cout << "SteadyStateVoltageRotation"; break;
		case PigeonDrift:                 std::cout << "PigeonDrift               "; break;
	}

	std::cout << (ms_ui_setting == VoltageSetting ? " *Voltage*: " : "  Voltage : ") << ms_voltage_setting << "V ";
	std::cout << (ms_ui_setting == MaxDistanceSetting ? "*MaxDist*: " : " MaxDist : ");

	switch (ms_type_setting) {
		case SteadyStateVoltage:          std::cout << ms_test_duration_setting << " feet\n"; break;
		case SteadyStateVoltageRotation:  std::cout << (ms_test_duration_setting/4.0) << " rotations\n"; break;
		case PigeonDrift:                 std::cout << (10*ms_test_duration_setting) << " seconds\n"; break;
	}

}



