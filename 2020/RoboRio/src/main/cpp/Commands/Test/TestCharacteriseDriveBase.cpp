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



// Timeout if the command ever takes more than 20s
#define TIMEOUT_S 20

//==============================================================================

TestCharacteriseDriveBase::UISetting TestCharacteriseDriveBase::ms_ui_setting = TestCharacteriseDriveBase::TypeSetting;
bool TestCharacteriseDriveBase::ms_pov_down = false;
TestCharacteriseDriveBase::Type TestCharacteriseDriveBase::ms_type_setting = TestCharacteriseDriveBase::SteadyStateVoltage;
double TestCharacteriseDriveBase::ms_voltage_setting = 3.0;
double TestCharacteriseDriveBase::ms_max_distance_feet_setting = 6.0;
TestCharacteriseDriveBase* TestCharacteriseDriveBase::ms_command = NULL;

//==============================================================================

TestCharacteriseDriveBase::TestCharacteriseDriveBase(Type type, double voltage, double maximum_distance_feet) :
	frc::Command("TestCommand", TIMEOUT_S) {

	m_type = type;
	m_voltage = voltage;
	m_maximum_distance_feet = maximum_distance_feet;

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());
}

TestCharacteriseDriveBase::~TestCharacteriseDriveBase() {
}

//==============================================================================
// Function Overrides from frc::Command

void TestCharacteriseDriveBase::Initialize() {
	// Drive the robot with the set open loop voltage
	DriveBase& drive_base = DriveBase::GetInstance();
	double output = m_voltage / 12.0;
	drive_base.TankDriveOpenLoop(output, output);

	// Clear the list of samples
	m_sample_list.clear();

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
	sample.m_velocity_fps = drive_base.GetVelocityFeetPerSecond();
	m_sample_list.push_back(sample);
}


bool TestCharacteriseDriveBase::IsFinished() {

	DriveBase& drive_base = DriveBase::GetInstance();
	int current_distance_inch = drive_base.GetDistanceInch();

	//std::cout << "TestCharacteriseDriveBase distance: " << current_distance_inch << " inch";
	bool exceeded_max_distance = (current_distance_inch > m_maximum_distance_feet * 12.0);

	return exceeded_max_distance || IsTimedOut();
}

void TestCharacteriseDriveBase::End() {
	// Stop the robot
	DriveBase& drive_base = DriveBase::GetInstance();
	drive_base.Stop();

	// Write the results to a file
	WriteResultsToFile();

	// Display the results in a simple form in the log
	int i = 0;
	int total_velocities = m_sample_list.size();
	while (i < total_velocities) {
		int limit = i + 10;
		if (limit > total_velocities) limit = total_velocities;
		while (i < limit) {
			std::cout << " [" << std::setprecision(3) << m_sample_list[i].m_time_s << " "  << m_sample_list[i].m_velocity_fps << " "  << m_sample_list[i].m_voltage << "]";
			i++;
		}
		std::cout << "\n";
	}
}

void TestCharacteriseDriveBase::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

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

	// Write the nominal test veloicty
	results_file << "\"Test Voltage\"," << m_voltage << "\n";

	// Write the sample times in a single line
	results_file << "\"Time(s)\"";
	int total_samples = m_sample_list.size();
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_time_s;
	results_file << "\n";

	// Write the sample voltages in a single line
	results_file << "\"Voltage(V)\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_voltage;
	results_file << "\n";

	// Write the sample velocities in a single line
	results_file << "\"Velocity (fps)\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_velocity_fps;
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
	const double DISTANCE_STEP = 1.0;

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
					case MaxDistanceSetting: SetMaxDistanceFeetSetting(ms_max_distance_feet_setting + DISTANCE_STEP); break;
				}
				break;
			case 180:
				// POV Down - Decrease the currently selected UI setting
				switch (ms_ui_setting) {
					case TypeSetting: 	  	 SetTypeSetting((Type)(ms_type_setting - 1)); break;
					case VoltageSetting:     SetVoltageSetting(ms_voltage_setting - VOLTAGE_STEP); break;
					case MaxDistanceSetting: SetMaxDistanceFeetSetting(ms_max_distance_feet_setting - DISTANCE_STEP); break;
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
    	ms_command = new TestCharacteriseDriveBase(SteadyStateVoltage, ms_voltage_setting, ms_max_distance_feet_setting);
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

void TestCharacteriseDriveBase::SetMaxDistanceFeetSetting(double distance_feet)
{
	const double MIN_DISTANCE_FEET = 3.0;
	const double MAX_DISTANCE_FEET = 30.0;
	if (distance_feet < MIN_DISTANCE_FEET) distance_feet = MIN_DISTANCE_FEET;
	if (distance_feet > MAX_DISTANCE_FEET) distance_feet = MAX_DISTANCE_FEET;
	ms_max_distance_feet_setting = distance_feet;
	DisplayConfigInLog();
}

void TestCharacteriseDriveBase::DisplayConfigInLog()
{
	std::cout << "TestCharacteriseDriveBase Config - ";
	std::cout << (ms_ui_setting == TypeSetting ? "*Type*: " : " Type : ");
	switch (ms_type_setting) {
		case SteadyStateVoltage: std::cout << "SteadyStateVoltage"; break;
		case Acceleration:       std::cout << "Acceleration      "; break;
		case QuasiStaticVoltage: std::cout << "QuasiStaticVoltage"; break;
	}
	std::cout << (ms_ui_setting == VoltageSetting ? " *Voltage*: " : "  Voltage : ") << ms_voltage_setting << "V ";
	std::cout << (ms_ui_setting == MaxDistanceSetting ? "*MaxDist*: " : " MaxDist : ") << ms_max_distance_feet_setting << " feet\n";
}



