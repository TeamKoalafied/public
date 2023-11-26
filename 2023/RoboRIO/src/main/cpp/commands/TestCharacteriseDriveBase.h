//==============================================================================
// TestCharacteriseDriveBase.h
//==============================================================================

#pragma once

#include "IAutonomousProvider.h"

#include "../util/Logging.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>


namespace frc { class SimpleWidget; }
class SwerveDrivebase;


// Command for performing drive base characterisation tests. The test configuration
// is read from the dashboard.
class TestCharacteriseDriveBase : public frc2::CommandHelper<frc2::CommandBase, TestCharacteriseDriveBase> {
public:
    //==========================================================================
    // Nested Types

    // Provider class for the TestCharacteriseDriveBase command
    class Provider : public IAutonomousProvider {
    public:
        virtual const char* Name() override {
            return "Test Characterise Drive Base";
        }

        virtual int SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) override {
            TestCharacteriseDriveBase::SetupDashboard();
            return 0;
        }

        virtual frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) override {
            return frc2::CommandPtr(std::unique_ptr<frc2::CommandBase>(new TestCharacteriseDriveBase(&drivebase)));
        }
    };


    //==========================================================================
    // Construction

    // Constructor
    //
    // drivebase - Drivebase to test
    TestCharacteriseDriveBase(SwerveDrivebase* drivebase);


    //==========================================================================
    // Function Overrides from frc2::CommandBase
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interupted) override;
    //==========================================================================


    //==========================================================================
    // Static Setup

    // Setup the dashbaord controls
    static void SetupDashboard();

private:
    //==========================================================================
    // Private Nested Types

    // The type of tests that can be performed
    enum Type {
        SteadyStateVoltage,             // Test of linear motion with a steady state voltage
        SteadyStateVoltageRotation,     // Test of rotation with a steady state voltage
        PigeonDrift,                    // Test of pigeon heading drift
    };

    // Build a single structure to keep all the mappings in one place to reduce risk of errors
    struct TypeMapping {
        std::string label;
        TestCharacteriseDriveBase::Type type;
    };

    // Shuffleboard widgets for adjusting test parameters
    struct ShuffleboardWidgets {
        // Smart dashboard chooser for the autonomouse strategy
        frc::SendableChooser<TestCharacteriseDriveBase::Type> m_type_chooser;

        frc::SimpleWidget* m_voltage;       // Widget for entering the voltage (V)
        frc::SimpleWidget* m_distance;      // Widget for entering the distance (m)
        frc::SimpleWidget* m_rotations;     // Widget for entering the rotations (whole rotations)
        frc::SimpleWidget* m_time;          // Widget for entering the time (s)
    };


    //==========================================================================
    // Member Variables

    SwerveDrivebase* m_drivebase;               // Drivebase to test
    Type m_type;                                // Type of test to perform
    units::volt_t m_voltage;                    // Voltage to apply for the test
    units::meter_t m_test_distance_m;           // Distance to run the SteadyStateVoltage in metres
    units::degree_t m_test_rotations;           // Number of rotations to run the SteadyStateVoltageRotation test for
    units::second_t m_test_time_s;              // Time to run the PigeonDrift test for in seconds

    Logging::CsvFile m_csv_log_file;            // CSV log file being written to
    bool m_aligned;                             // Whether the drivebase wheels are aligned as required for the test
    frc::Timer m_timer;                         // A timer that measures elapsed time
    units::degree_t m_initial_heading;          // Heading when the command starts (used for rotation test)

    static ShuffleboardWidgets* ms_shuffleboard_widgets;
                                                // Shuffleboard widgets for adjusting test parameters. Null if not initialised.
    static const TypeMapping kTypeMappings[];   // Define the mapping between the labels on the dashboard and test type
    static const int kTypeMappingsCount;        // Number of entries for the test type

};
