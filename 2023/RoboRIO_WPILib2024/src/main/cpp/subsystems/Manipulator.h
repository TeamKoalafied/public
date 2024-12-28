//==============================================================================
// Manipulator.h
//==============================================================================

#pragma once

#include "../Phoenix5Header.h"
#include "../util/HapticController.h"
#include "../util/PeriodicTimer.h"
#include "../util/PovFilter.h"

#include <frc/XboxController.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/RunCommand.h>
#include <frc/Timer.h>

#include <units/length.h>

namespace frc
{
class ShuffleboardTab;
class SimpleWidget;
}

class ManipulatorShuffleboard;
class Intake;
class Arm;
class Pivot;
class Wrist;
class NewIntake;
namespace Logging { class CsvFile; }



// The Manipulator subsystem controls all the operator parts of the robot. It consists of the
// following mechanisms.
//
//  - Pivot
//  - Arm
//  - Wrist
//  - Intake
class Manipulator : public frc2::SubsystemBase{
public:
    //==========================================================================
    // Public Nested Types

    // Positions for the manipulator to be in (pivot, arm, wrist)
    enum class Position {
        Unknown,
        TestCustom,             // Custom position for tuning positions during testing
        Starting,               // Completly stowed in the frame perimeter and height limit, as for starting the match
        Stowed,                 // Stowed safety in the frame perimeter, as for moving during the match 
        GroundConePickup,
        GroundCubePickup,
        ShelfConePickup,        // Picking up from the human player station
        ShelfCubePickup,        // Picking up from the human player station
        Level1ConePlace,
        Level1CubePlace,
        Level2ConePlace,
        Level2CubePlace,
        Level3ConePlace,
        Level3CubePlace,
        FirstValid = Stowed,
        LastValid = Level3CubePlace
    };

    // Types of game pieces
    enum class GamePiece {
        Cone,
        Cube
    };


    //==========================================================================
    // Construction

    // Constructor
    Manipulator();

    // Destructor
    virtual ~Manipulator();


    //==========================================================================
    // frc::Subsystem Function Overrides

    // Execute periodic function of each mechanism & update widgets
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the manipulator subsystem for operation
    void Setup();

    // Setup shuffleboard tab with summary data from mechanisms

    // Shutdown the manipulator subsystem
    void Shutdown();

    // Haptic controller for operator joystick
    HapticController* GetHapticController() { return m_haptic_controller; }

    // Initialise for teleop state
    void TeleopInit();

    // Initialise for disabled state
    void DisabledInit();



    //==========================================================================
    // Manipulator Control

    // Move the pivot/arm/wrist to the given position
    //
    // position - Position to move to
    void GoToPosition(Position position);

    // Opens the intake fully
    void DropGamePiece();

    // Stop the intake
    void StopIntake();


    //==========================================================================
    // Manipulator State

    // Check if the pivot/arm/wrist is at the given position, within a small limit
    //
    // position - Position check against
    //
    // Returns whether the pivot/arm/wrist is at the given position
    bool IsAtPosition(Position position, bool low_tolerance);

    // Get the type of game piece (cone/cube) that the intake is confrgure to grab
    GamePiece GetGamePiece() const { return m_game_piece; }

    // Set the type of game piece (cone/cube) that the intake is confrgure to grab
    void SetGamePiece(GamePiece game_piece) { m_game_piece = game_piece; }

    // Get whether the intake currently has grabbed a game piece
    bool HasGamePiece() { return m_has_game_piece; }

    // Get whether the intake is currently trying to grab a game piece
    bool IsIntaking() { return m_intaking; }

    // Get the distance that the armm is extended
    units::inch_t GetArmExtension();

    // Get the angle that the wrist is rotate by
    units::degree_t GetWristOffset() const;


    //==========================================================================
    // Autonomous Control

private:
    //==========================================================================
    // Private Nested Types


    // Pivot, arm and wrist values that define a position
    struct PositionInfo {
        Position m_position;            // Position being defined
        units::degree_t m_pivot_angle;  // Pivot angle for this position
        units::inch_t m_arm_extension;  // Arm angle for this position
        units::degree_t m_wrist_angle;  // Wrist angle for this position
    };


    //==========================================================================
    // Joystick Control

    // Control this subsystem with the joystick controller
    void DoJoystickControl();

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::XboxController* joystick);

    // Get the angle of the wrist for a given pivot angle that will keep the intake parallel to the ground
    //
    // pivot_angle - Pivot angle to calculate for
    //
    // Returns the required wrist angle
    units::degree_t GetWristAngleFromPivot(units::degree_t pivot_angle);

    // Set up csv filetream for logging arm extension, wrist & pivot angle
    void StartLoggingGotoPosition();

    // Add current values to log filoe
    void UpdateLoggingGotoPosition();

    // Close csv filestream 
    //
    // cancelled - Whether the movement was cancelled
    void StopLoggingGotoPosition(bool cancelled);

    // Convert current game piece (enum) to string
    constexpr const char* GamePieceToString(GamePiece gamepiec);

    // Convert current manipulator position (enum) to string
    constexpr const char* PositionToString(Position position);


    //==========================================================================
    // Position Control

    // Get the pivot, arm and wrist values that define a position
    //
    // position - Position to get the definition for
    //
    // Returns an info struct defining the position
    const PositionInfo& GetPositionInfo(Position position);

    // Get the custom pivot, arm and wrist values that define a position
    //
    // Returns an info struct defining the position
    const PositionInfo& GetCustomPositionInfo();

    // Get the current position of the manipulator. Note that if not at one of the defined
    // positions this will return Unknown.
    //
    // Returns a defined position, or Unknown
    Position GetCurrentPosition();

    //==========================================================================
    // Private Nested Types


    //==========================================================================
    // Member Variables

    Pivot* m_pivot;             // Pivot mechanism
    NewIntake* m_intake;        // Intake mechanism (new version)
    Arm* m_arm;                 // Arm mechanism
    Wrist* m_wrist;             // Wrist mechanism
   
    frc::XboxController* m_controller;              // XBox controller
    HapticController* m_haptic_controller;          // Haptic controller for operator joystick
    PovFilter m_pov_filter;                         // Filter for POV so that it only returns the four major directions


    Position m_manual_position = Position::Unknown; // Current position of the manipulator
    bool m_manual_position_complete = false;        // Value for whether manipulator is at current desired position
    units::degree_t m_wrist_manual_offset = 0_deg;  // Offset value for wrist encoder
    GamePiece m_game_piece = GamePiece::Cone;       // Which game piece is currently being handled
    bool m_has_game_piece = false;                  // Whether we have a game piece at the moment
    frc::Timer m_position_timer;                    // Timer for determining final position movement
    bool m_at_position_lower_tolerance = false;     // 

    frc::Timer m_intake_timer;                      // Timer for run the intake is running
    units::inch_t m_intake_initial_arm_extension;

    ManipulatorShuffleboard* m_shuffleboard = nullptr;      // Shuffleboard controller for this manipulator subsystem

    PositionInfo m_custom_position_info;                    // Custom position for tuning positions

    Logging::CsvFile* m_goto_position_csv_log_file = nullptr;   // Logging file for gotoposition
    frc::Timer m_goto_position_timer;                           // Timer for current time step in log file
    double m_goto_position_pivot_time;                          // Time taken for pivot to reach desired position
    double m_goto_position_arm_time;                            // Time taken for arm to reach desired position
    double m_goto_position_wrist_time;                          // Time taken for wrist to reach desired position

    bool m_intaking = false;                                // Contains whether intake is running (used to enable/disable logging)

    PeriodicTimer m_joystick_timer;
    PeriodicTimer m_simulation_timer;
    PeriodicTimer m_periodic_timer;


    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
};
