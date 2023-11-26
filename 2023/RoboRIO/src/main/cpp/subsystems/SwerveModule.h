//==============================================================================
// SwerveModule.h
//==============================================================================

#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/DoubleTopic.h>
#include <units/voltage.h>
#include <units/current.h>

#include "../util/GyroSendable.h"
#include "../pathfollower/SwerveFollowerModuleState.h"

// SwerveModule handles control one of the for swerve modules on the robot.
// This class handles the following:
//
//  - Get/set the 'state' of the module, that is its speed and direction
//  - Setting the 'calibration' of the angle (CANcoder angle when the module is straight)
//  - Displaying data on the shuffle board
//  - Simulation of the module
//  - Test operations used for debugging and motor tuning
class SwerveModule {
public:
    //==========================================================================
    // Contruction

    // Constructor
    //
    // name - Name of the module, for logging and dashboard
    // drive_falcon_id - CAN id of the Falcon FX controller for the drive motor
    // steer_falcon_id - CAN id of the Falcon FX controller for the steering motor
    // steer_encoder_id - CAN id of the CANCoder encoder for the steering wheel
    // zero_point - Encoder angle when the module is at zero
    SwerveModule(std::string_view name, int drive_falcon_id, int steer_falcon_id, int steer_encoder_id, units::radian_t zero_point);


    //==========================================================================
    // Operations

    // Get the current state (speed & angle) of this module
    frc::SwerveModuleState GetState() const;

    // Get the current position (distance & angle) of this module
    frc::SwerveModulePosition GetPosition() const;

    // Set the current state (speed & angle) of this module
    void SetDesiredState(const frc::SwerveModuleState& state, bool dry_steer_allowed);

    // Set the current follower state (acceleration, speed & angle) of this module
    void SetDesiredFollowerState(const SwerveFollowerModuleState& state);

    // Get the current absolute steer encoder position in degrees
    units::degree_t GetAbsoluteSteerEncoderPosition();

    // Set the calibration for steering
    //
    // absolute_encoder_zero_position - Absolute steer encoder position in degrees when the
    //      wheel is steering to 0 angle (straight ahead)
    void SetSteerEncoderCalibration(units::degree_t absolute_encoder_zero_position);

    // Set the brake/coast mode for the drive motor
    //
    // brake - Whether to set brake mode (otherwise coast)
    void SetBrakeMode(bool brake);


    //==========================================================================
    // Shuffleboard

    // Adds a layout containing summary data for a swerve module to shuffleboard_tab
    //
    // shuffleboard_tab - tab that the layout will be added to
    // x_pos - x position of the top left corner of the layout in the tab
    // y_pos - y position of the top left corner of the layout in the tab
    void CreateShuffleboardSummary(frc::ShuffleboardTab& shuffleboard_tab, int x_pos, int y_pos) const;

    // Adds a gyro sendable for the module to shuffleboard_tab
    //
    // shuffleboard_tab - tab to add the gyro sendable to
    // x_pos - x position of the top left corner of the layout in the tab
    // y_pos - y position of the top left corner of the layout in the tab
    void AddGyroSendable(frc::ShuffleboardTab& shuffleboard_tab, int x_pos, int y_pos) const;

    // Creates a shuffleboard tab including detailed data for a swerve module
    //
    // name - name of the shuffleboard tab being created
    void CreateShuffleboardDetailedTab(std::string_view name) const;

    // Update the shuffleboard display for this module
    void UpdateShuffleboard() const;


    //==========================================================================
    // Simulation Operations

    // Update the simulation of the module (for desktop simulation)
    //
    // update_time - Time increment to update for
    void UpdateSimulation(units::second_t update_time);


    //==========================================================================
    // Test Operations

    // Run the drive controller in either open or close loop and display important parameters on
    // the smart dashboard for tuning of the PID 
    //
    // drive - Proportional drive from -1 to 1
    // max_rpm - RPM value that corresponds to full drive for close loop
    // close_loop - Whether to drive in close loop velocity control (otherwise open loop)
    void TuneDriveTalonFX(double drive, double max_rpm, bool close_loop);

    // Run the steer motorin either open or close loop and display important parameters on
    // the smart dashboard for tuning of the PID 
    //
    // drive - Proportional drive from -1 to 1
    // max_rpm - RPM value that corresponds to full drive for close loop
    // close_loop - Whether to drive in close loop velocity control (otherwise open loop)
    void TuneSteerTalonFX(double drive, double max_rpm, bool close_loop);

    // Drive the motors using percentage output for testing
    void ManualDriveMotors(double drive_fraction, double steer_fraction);

    // Set the current state (speed & angle) of this module
    void CharacterisationDrive(const frc::Rotation2d& angle, double drive_fraction);

    // Reset the steering Falcon encoder to 0
    void ResetSteerEncoder();

    units::volt_t GetDriveVoltage() const;
    units::ampere_t GetDriveCurrent() const;

private:
    //==========================================================================
    // Motor Controller Setup

    // Creates and loads the configuration of the steer TalonFX controller
    void SteerTalonSetup();

    // Creates and loads the configuration of the drive TalonFX controller
    void DriveTalonSetup();
    
    //==========================================================================
    // Steering Control
    
    // Moves the steering motor from the current angle to a target angle using motion magic
    //
    // angle - target steering angle
    void SteerToAngleMotionMagic(units::radian_t angle);

    //==========================================================================
    // Private nested types

    // Shuffleboard widgets contained in the detailed tab   
    struct DetailedWidgets {

        frc::SimpleWidget* m_absolute_encoder_widget;   // Widget displaying the absolute encoder angle
        frc::SimpleWidget* m_steer_angle_module_widget; // Widget displaying the steer angle relative to the module
        frc::SimpleWidget* m_zero_point_widget;         // Widget displaying the zero point

        frc::SimpleWidget* m_desired_speed_widget;      // Widget displaying the modules target speed
        frc::SimpleWidget* m_desired_angle_widget;      // Widget displaying the modules target heading

        frc::SimpleWidget* m_heading_graph_widget;      // Widget for the graph of module heading over time

        frc::SimpleWidget* m_speed_widget;              // Widget displaying the current wheelspeed 
        frc::SimpleWidget* m_speed_graph;               // Widget graphing the current wheelspeed
        frc::SimpleWidget* m_steer_current_graph;       // Widget graphing the steer motor current over time
        frc::SimpleWidget* m_drive_current_graph;       // Widget graphing the steer motor current over time

        frc::SimpleWidget* m_steer_output_widget;       // Widget displaying the current output percentage of the steer motor
        frc::SimpleWidget* m_steer_output_graph_widget; // Widget graphing the percentage output of the steer motor over time
        frc::SimpleWidget* m_steer_position_widget;     // Widget displaying the current encoder position of the steer motor
        frc::SimpleWidget* m_drive_position_widget;     // Widget displaying the current encoder position of the drive motor
    };

    //==========================================================================
    // Member Variables

    std::string m_name;                             // Name of the module, for logging and dashboard
    mutable WPI_TalonFX m_drive_controller;         // Talon FX drive speed controller
    mutable WPI_TalonFX m_steer_controller;         // Talon FX steer speed controller
    mutable WPI_CANCoder* m_steer_encoder;          // CAN id of the CANCoder encoder for the steering wheel

    units::radian_t m_zero_point;                   // Absolute encoder angle when the module is at zero angle

    // frc::ProfiledPIDController<units::radians> m_turning_pid_controller;
    // frc::SimpleMotorFeedforward<units::radians> m_turn_feedforward;

    frc::SimpleWidget* m_canbus_error_widget;       // Shuffleboard widget that indicates the presence of a CAN bus error
    bool m_can_error;                               // Contains whether there is currently a CAN bus error

    mutable GyroSendable m_heading_sendable;                // Shuffleboard sendable containing the current heading of the module
    mutable frc::SimpleWidget* m_drive_speed_widget;        // Shuffleboard widget displaying the drive motor speed

    mutable frc::SimpleWidget* m_drive_current_widget;      // Shuffleboard widget displaying the drive motor current
    mutable frc::SimpleWidget* m_drive_temp_widget;         // Shuffleboard widget displaying the drive motor temperature
   
    mutable frc::SimpleWidget* m_steer_current_widget;      // Shuffleboard widget displaying the steer motor current
    mutable frc::SimpleWidget* m_steer_temp_widget;         // Shuffleboard widget displaying the steer motor temperature

    mutable DetailedWidgets* m_detailed_widgets = nullptr;  // Struct containing shuffleboard widgets for the detailed tab

    mutable frc::SwerveModuleState m_desired_state;         // Record of the desired state set in the SetDesiredState() function
};

