//==============================================================================
// ManipulatorShuffleboard.h
//==============================================================================

#pragma once

#include "Manipulator.h"
class Arm;
class NewIntake;
class Pivot;
class Wrist;

const double DEFAULT_CONE_SPEED = 1.0;
const double DEFAULT_CUBE_SPEED = -0.7;
const double DEFAULT_CONE_CURRENT = 16;
const double DEFAULT_CUBE_CURRENT = 13;
const double DEFAULT_CONE_TIMEOUT = 0.3;
const double DEFAULT_CUBE_TIMEOUT = 0.0;

class ManipulatorShuffleboard {
public:
    // Public structs used to bulk-return modifiable values from the shuffleboard to where they are used in the manipulator
    
    // Contains values from intake tuning widgets to control current limits & timeouts, and roller speeds
    struct IntakeParams {
        double cone_current;
        double cone_speed;
        double cone_timeout;
        double cube_current;
        double cube_speed;
        double cube_timeout;
    }; 

    // Contains position data for moving to the custom tunable position on the shuffleboard
    struct PositionParams {
        units::degree_t pivot_angle;
        units::degree_t wrist_angle;
        units::inch_t arm_extension;
    };
    
    // Constructor that takes const pointers to the manipulator and all its mechanisms
    //
    // manipulator  - const pointer to the manipulator
    // arm          - const pointer to the arm
    // intake       - const pointer to the current (new) intake
    // pivot        - const pointer to the pivot
    // wrist        - const pointer to the wrist
    ManipulatorShuffleboard(const Manipulator* manipulator, const Arm* arm, const NewIntake* intake, 
                            const Pivot* pivot, const Wrist* wrist);

    // Setup called in manipulator setup and creates/populates tabs by calling helpers
    void Setup();

    // Updates all existing readout tabs with current values
    void Update();

    // Return current values for the intake tuning
    IntakeParams* GetIntakeTune();

    // Return current values for the current position parameters
    PositionParams* GetCustomPosition();
    void UpdateCustomPosition(PositionParams pos);

private:
    //==========================================================================
    // Private Nested Types

    // Struct containing widgets contained in the arm layout
    struct ArmWidgets
    {   
        frc::SimpleWidget* m_forward_limit_widget;
        frc::SimpleWidget* m_reverse_limit_widget;
        frc::SimpleWidget* m_extension_widget;
        frc::SimpleWidget* m_current_widget;
    };

    // Struct containing widgets contained in the wrist layout
    struct WristWidgets
    {   
        frc::SimpleWidget* m_forward_limit_widget;
        frc::SimpleWidget* m_reverse_limit_widget;
        frc::SimpleWidget* m_angle_widget;
        frc::SimpleWidget* m_current_widget;
        frc::SimpleWidget* m_wrist_offset_widget;
    };

    // Struct containing widgets contained in the intake layout
    struct IntakeWidgets
    {   
        frc::SimpleWidget* m_roller_current_widget;
    };

    // Struct containing widgets contained in the pivot layout
    struct PivotWidgets
    {   
        frc::SimpleWidget* m_forward_limit_widget;
        frc::SimpleWidget* m_reverse_limit_widget;
        frc::SimpleWidget* m_angle_widget;
        frc::SimpleWidget* m_current_widget;
    };

    // Struct containing widgets for tuning the custom position
    struct TunePositionWidgets {   
        frc::SimpleWidget* m_pivot_angle_widget;
        frc::SimpleWidget* m_arm_extension_widget;
        frc::SimpleWidget* m_wrist_angle_widget;
    };

    // Struct containing widgets for tuning the intake 
    struct IntakeTuningWidgets 
    {
        frc::SimpleWidget* m_cone_speed;
        frc::SimpleWidget* m_cube_speed;
        frc::SimpleWidget* m_cone_current;
        frc::SimpleWidget* m_cube_current;
        frc::SimpleWidget* m_cone_timeout;
        frc::SimpleWidget* m_cube_timeout;
    };
    
    // Helper methods that isolate setup for each mechanism/tuner
    // Sets up relevant widgets in a layout on a specified tab
    // 
    // tab - shuffleboard tab to add the layout to
    // x - x-coordinate of left edge of the layout on 'tab'
    // y - y-coordinate of top edge of the layout on 'tab'
    void SetupArm(frc::ShuffleboardTab& tab, int x, int y);
    void SetupPivot(frc::ShuffleboardTab& tab, int x, int y);
    void SetupWrist(frc::ShuffleboardTab& tab, int x, int y);
    void SetupIntake(frc::ShuffleboardTab& tab, int x, int y);
    void SetupTunePosition(frc::ShuffleboardTab& tab, int x, int y);
    void SetupTuneIntake(frc::ShuffleboardTab& tab, int x, int y);


    const Manipulator* m_manipulator;
    const Arm* m_arm;
    const NewIntake* m_intake;
    const Pivot* m_pivot;
    const Wrist* m_wrist;

    ArmWidgets* m_arm_widgets = nullptr;
    WristWidgets* m_wrist_widgets = nullptr;
    IntakeWidgets* m_intake_widgets = nullptr;
    IntakeTuningWidgets* m_intake_tune_widgets = nullptr;
    PivotWidgets* m_pivot_widgets = nullptr;
    TunePositionWidgets* m_tune_position_widgets = nullptr;
};
