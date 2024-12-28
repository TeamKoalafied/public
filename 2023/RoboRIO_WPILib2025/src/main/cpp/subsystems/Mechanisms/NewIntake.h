//==============================================================================
// NewIntake.h
//==============================================================================

#pragma once

#include "../../Phoenix5Header.h"

#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}
namespace Logging { class CsvFile; }


// The Intake mechanism is part of the Manipulator subsystem. It grabs the cones and cubes.
// The intake consists of the following actuators and sensors:
//
//  - 775pro Roller motor controlled by a Talon SRX
// 
class NewIntake  {
public:
    //==========================================================================
    // Construction

    // Constructor
    NewIntake();

    // Destructor
    ~NewIntake();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates for the Intake
    void Periodic();


    //==========================================================================
    // Operations

    // Run the intake for grabbing cones
    void IntakeCone(double current_limit, double speed, double timeout);

    // Run the intake for grabbing cubes
    void IntakeCube(double current_limit, double speed, double timeout);

    // Get whether the game piece has been fully grabbed. Only valid when intaking.
    bool HasGamePiece();

    // Drop the a cone by running the rollers in reverse
    void DropCone();

    // Drop the a cube by running the rollers in reverse
    void DropCube();

    // Stop running the intake for cones
    void Stop();

    double GetCurrent() const;
    //==========================================================================
    // Shuffleboard

    // Set up csv filestream for logging intake values
    //
    // game_piece - Name of the game piece to include in the log
    void StartLoggingIntake(const char* game_piece);

    // Add current intake state values to logfile
    void UpdateLoggingIntake();

    // Close csv filestream
    void StopLoggingIntake();


    //==========================================================================
    // Testing

    // Manually drive the intake at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is capture balls.
    void ManualDriveIntake(double percentage_output);

    // Perform testing of the intake using the joystick. This function is only for testing the
    // intake and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveIntake(frc::XboxController* joystick);

private:
    //==========================================================================
    // Private Nested Types
    
    // Widgets for the normal summary shuffleboard

    // States for intaking state machine
    enum class IntakingState {
        Idle,
        Intaking,           // Intaking game piece without hitting the high current limit
        HighCurrent,        // Intaking and have hit high current, but timer has not expired
        HasGamePiece,       // Hit high currnt and timer expired, so we have the game piece
    };


    //==========================================================================
    // Member Variables

    TalonSRX* m_intake_speed_controller;  // Motor controller for the intake roller
    frc::Timer m_intake_current_timer;      // Timer for the intake motor high current detection

    IntakingState m_intake_state = IntakingState::Idle;          

    Logging::CsvFile* m_intake_csv_log_file = nullptr;      // Logging file for intake
    frc::Timer m_intake_timer;                              // Timer for current time step in intake log file
};
