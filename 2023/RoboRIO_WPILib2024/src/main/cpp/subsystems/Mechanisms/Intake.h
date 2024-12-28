//==============================================================================
// Intake.h
//==============================================================================

#ifndef Intake_H

#include "../../Phoenix5Header.h"

#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Intake mechanism is part of the Manipulator subsystem. It grabs the cones and cubes.
// The intake consists of the following actuators and sensors:
//
//  - Roller motor controlled by a Talon SRX with magnetic encoder
// 
class Intake  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Intake();

    // Destructor
    ~Intake();


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

    enum class Position {
        Unknown,    // Has not fully openned
        OpenFully,
        OpenCone,
        OpenCube,
        ClosedFully,
        ClosedCone,
        ClosedCube
    };

    // Should probably be private
    void GoToPosition(Position position);

    bool IsAtPosition(Position position);

    double SeekEncoderForPosition(Position position);

    // Run the intake for grabbing cones
    void RunCone();

    // Run the intake for grabbing cubes
    void RunCube();

    // Drop the current game piece by fully opening the intake, without running the rollers
    void DropGamePiece();

    // Stop running the intake for cones
    void StopCone();

    // Stop running the intake for cubes
    void StopCube();


    // Manually drive the intake at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is capture balls.
    void ManualDriveIntake(double percentage_output);

    // Manually drive the intake at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is capture balls.
    void ManualDriveWinch(double percentage_output);

    // Check if current is exceeding the limit (12.5A)
    bool HasHighCurrent();


    // Perform testing of the intake using the joystick. This function is only for testing the
    // intake and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
   void TestDriveIntake(frc::XboxController* joystick);

    // Create layout for summary shuffleboard data
    //
    // shuffleboard_tab - tab that the layout will be added to
    // x_pos - x position of the top left corner of the layout in the tab
    // y_pos - y position of the top left corner of the layout in the tab
    void CreateShuffleboardSummary(frc::ShuffleboardTab& shuffleboard_tab, int x, int y);

    double GetRollerOutput();
    double GetRollerCurrent();
    double GetWinchPosition();
    double GetWinchCurrent();


private:
    //==========================================================================
    // Private Nested Types
    struct IntakeWidgets
    {   
        frc::SimpleWidget* m_winch_position_widget;
        frc::SimpleWidget* m_winch_current_widget;
        frc::SimpleWidget* m_winch_state_widget;
        frc::SimpleWidget* m_winch_encoder_widget;
        frc::SimpleWidget* m_roller_current_widget;
    };

    enum class WinchState {
        Seeking,
        Open,
        Openning,
        OpenCurrent,
        Unknown
    };

    //==========================================================================
    // Member Variables

    TalonSRX* m_intake_speed_controller;  // Motor controller for the intake roller
    TalonSRX* m_winch_controller;           // Controller for the winch motor that moves the intake sides
    frc::Timer m_intake_current_timer;      // Timer for the intake motor high current detection

    bool m_intake_high_current = false;     

    WinchState m_winch_state = WinchState::Unknown;
    double m_winch_seek_encoder = -10000;
    frc::Timer m_winch_current_timer;

    double m_closed_winch_encoder = 0.0;
    double m_open_winch_encoder = 0.0;

    IntakeWidgets* m_intake_widgets = nullptr;               
};

#endif  // Intake_H
