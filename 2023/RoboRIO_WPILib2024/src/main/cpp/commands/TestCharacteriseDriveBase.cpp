//==============================================================================
// TestCharacteriseDriveBase.cpp
//==============================================================================

#include "TestCharacteriseDriveBase.h"

#include "../subsystems/SwerveDrivebase.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <frc/geometry/Pose2d.h>
//#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>



namespace {


}

TestCharacteriseDriveBase::ShuffleboardWidgets* TestCharacteriseDriveBase::ms_shuffleboard_widgets = nullptr;

const TestCharacteriseDriveBase::TypeMapping TestCharacteriseDriveBase::kTypeMappings[] = {
    { "Steady State Voltage",          TestCharacteriseDriveBase::Type::SteadyStateVoltage },
    { "Steady State Voltage Rotation", TestCharacteriseDriveBase::Type::SteadyStateVoltageRotation },
    { "Pigeon Drift",                  TestCharacteriseDriveBase::Type::PigeonDrift },
};

const int TestCharacteriseDriveBase::kTypeMappingsCount = sizeof(TestCharacteriseDriveBase::kTypeMappings) /
                                                          sizeof(TestCharacteriseDriveBase::kTypeMappings[0]);


//==============================================================================

TestCharacteriseDriveBase::TestCharacteriseDriveBase(SwerveDrivebase* drivebase) {
    m_drivebase = drivebase;

    // Get the test type and parameters from the dashboard
    m_type = ms_shuffleboard_widgets->m_type_chooser.GetSelected();
    m_voltage = units::volt_t(ms_shuffleboard_widgets->m_voltage->GetEntry()->GetDouble(3.0));
    m_test_distance_m = units::meter_t(ms_shuffleboard_widgets->m_distance->GetEntry()->GetDouble(3.0));
    m_test_rotations = units::degree_t(ms_shuffleboard_widgets->m_rotations->GetEntry()->GetDouble(3.0) * 360.0);
    m_test_time_s = units::second_t(ms_shuffleboard_widgets->m_time->GetEntry()->GetDouble(60.0));

    // Clip the parameters to sensible values
    m_voltage = std::clamp(m_voltage, 1.0_V, 12.0_V);
    m_test_distance_m = std::clamp(m_test_distance_m, 2.0_m, 10.0_m);
    m_test_rotations = std::clamp(m_test_rotations, 360_deg, 10.0 * 360_deg);
    m_test_time_s = std::clamp(m_test_time_s, 2.0_s, 600.0_s);

    // Driving requires the DriveBase
    AddRequirements({m_drivebase});
}


//==============================================================================
// Function Overrides from frc::Command

void TestCharacteriseDriveBase::Initialize() {
    // Reset the drive base pose
    m_drivebase->ResetPose(frc::Pose2d());

    // Open the CSV log file
    const char* const RESULT_FILENAME = "CharacteriseDriveBase.csv";
    m_csv_log_file.OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_csv_log_file.Fail()) {
        std::cout << "TestCharacteriseDriveBase::Initialize() - Failed to open log file\n";
        return;
    }

    // Add the heading for the test parameters to the log file
    m_csv_log_file << "Characterisation Parameters" << "\n";

    // Write parameters of the test and perform initialisation, depending on which test is
    // being run
    switch (m_type) {
        case SteadyStateVoltage:
            m_csv_log_file << "Test Type" << "SteadyStateVoltage" << "\n";
            m_csv_log_file << "Test Voltage" << m_voltage.value() << "\n";
            m_aligned = false;
            break;
        case SteadyStateVoltageRotation:
            m_csv_log_file << "Test Type" << "SteadyStateVoltageRotation" << "\n";
            m_csv_log_file << "Test Voltage" << m_voltage.value() << "\n";
            m_aligned = false;
            break;
        case PigeonDrift:
            m_csv_log_file << "Test Type" << "PigeonDrift" << "\n";
            m_drivebase->Stop();
            m_aligned = true;
            break;
    }
    m_initial_heading = m_drivebase->GetPigeonHeading();
    m_csv_log_file << "\n";

    // Write the CSV header row and set the precision for numbers
    m_csv_log_file << "Characterisation Data" << "\n";
    m_csv_log_file << "Time (s)" << "Voltage (V)" << "Velocity (fps)" << "Heading (degrees)" << "Current (A)" << "\n";
    m_csv_log_file.SetPrecision(3, true);

    // Restart the timer
    m_timer.Reset();
    m_timer.Start();
}

void TestCharacteriseDriveBase::Execute() {
    units::volt_t MAX_VOLTAGE = 12_V;
    switch (m_type) {
        case SteadyStateVoltage:
            // If the wheel are not aligned turn them until they are
            if (!m_aligned) {
                m_drivebase->AlignWheelsToDrive(1_mps, 0_mps, false);
                if (m_drivebase->AreWheelsAlignedToDrive(1_mps, 0_mps, false)) {
                    // Wheels are now aligned so start the test from now
                    m_aligned = true;
                    m_timer.Reset();
                    m_drivebase->CharacterisationDrive(m_voltage/MAX_VOLTAGE, 0, 0_rad_per_s, false);
                } else {
                    // We are not align so the test had not started yet. Bail out
                    // so that we don't log anything yet.
                    return;
                }
            } else {
                m_drivebase->CharacterisationDrive(m_voltage/MAX_VOLTAGE, 0, 0_rad_per_s, false);
            }
            break;
        case SteadyStateVoltageRotation:
            // if (!m_aligned) {
            //     m_drivebase->AlignWheelsToDrive(1_mps, 0_mps, false);
            //     if (m_drivebase->AreWheelsAlignedToDrive(1_mps, 0_mps, false)) {
            //         m_aligned = true;
            //         m_timer.Reset();
            //     } else {
            //         return;
            //     }
            // } else {
            //     m_drivebase->CharacterisationDrive(m_voltage/12, 0, 0_rad_per_s, false);
            // }
            break;
        case PigeonDrift:
            break;
    }

    // Get the current state and write it to the result log file. Use the pigeon angle
    // for measuring rotation as it 'winds up' and allows us to measure multiple rotations
    // easily.
    frc::ChassisSpeeds chassis_speed = m_drivebase->GetChassisSpeeds();
    units::feet_per_second_t speed = chassis_speed.vx;
    wpi::array<const SwerveModule*,4> modules = m_drivebase->GetModules();
    units::volt_t voltage = modules[0]->GetDriveVoltage();
    units::ampere_t current  = modules[0]->GetDriveCurrent();
    units::degree_t angle = m_drivebase->GetPigeonHeading() - m_initial_heading;
    m_csv_log_file << m_timer.Get().value() << ::fabs(voltage.value()) << speed.value() << angle.value() << current.value() << "\n";
}

bool TestCharacteriseDriveBase::IsFinished() {
    // Determine if the test is done depending on what test is being performed
    bool test_done = false;
    switch (m_type) {
        case SteadyStateVoltage: {
            // Test duration is a distance in feet
            const frc::Pose2d& pose = m_drivebase->GetPose();
            units::meter_t current_distance = pose.Translation().Norm();
            test_done = current_distance > m_test_distance_m;
            break;
        }
        case SteadyStateVoltageRotation:
            // Test duration is a number of rotations
            break;
        case PigeonDrift:
            // Nothing to do. The timeout controls the length of the test.
            break;
    }

    // Also apply an overall timeout just in case
    const units::second_t TIMEOUT = 10_s;
    return test_done || m_timer.Get() > TIMEOUT;
}

void TestCharacteriseDriveBase::End(bool interupted) {

    // Stop the drivebase
    m_drivebase->Stop();

    // Add a blank line to the log file to separate from any other data that may be appended
    m_csv_log_file << "\n";

    // Close the log file. This is important as this class may not be destroyed for some time.
    m_csv_log_file.Close();
}


//==============================================================================
// Static Setup

// Setup the dashbaord controls
void TestCharacteriseDriveBase::SetupDashboard() {
    // Create a enw tab to put the controls on
    frc::ShuffleboardTab& characterisation_tab = frc::Shuffleboard::GetTab("Characterise");
    ms_shuffleboard_widgets = new ShuffleboardWidgets;
    ShuffleboardWidgets* sw = ms_shuffleboard_widgets;

    // Setup a drop down for choosing the type of characterisation to run
    for (int i = 0; i < kTypeMappingsCount; i++) {
        ms_shuffleboard_widgets->m_type_chooser.AddOption(kTypeMappings[i].label, kTypeMappings[i].type);
    } 
    ms_shuffleboard_widgets->m_type_chooser.SetDefaultOption(kTypeMappings[0].label, kTypeMappings[0].type);
    characterisation_tab.Add("Auto Type", ms_shuffleboard_widgets->m_type_chooser).WithPosition(0, 0).WithSize(6, 3);

    // Add the widgets for the test parameters with reasonable starting default values
    sw->m_voltage = &characterisation_tab.Add("Characterisation Voltage (V)", 3.0).WithPosition(0,3).WithSize(6,3);
    sw->m_distance = &characterisation_tab.Add("Characterisation Distance (m)", 3.0).WithPosition(6,3).WithSize(6,3);
    sw->m_rotations = &characterisation_tab.Add("Characterisation Rotations", 3.0).WithPosition(0,6).WithSize(6,3);
    sw->m_time = &characterisation_tab.Add("Characterisation Time (s)", 3.0).WithPosition(6,6).WithSize(6,3);
}
