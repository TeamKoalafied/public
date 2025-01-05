//==============================================================================
// ManipulatorShuffleboard.h
//==============================================================================

#pragma once

#include "Manipulator.h"
class Intake;
class Shooter;
class Diverter;

class ManipulatorShuffleboard {
public:
    struct TuningValues {
        units::degree_t pivot_angle;
        units::revolutions_per_minute_t shooter_rpm;
        double intake_drive;
        double diverter_drive;
        double trap_drive;
        units::inch_t arm_target;
    };
    // Constructor that takes const pointers to the manipulator and all its mechanisms
    //
    // manipulator  - const pointer to the manipulator
    ManipulatorShuffleboard(const Manipulator* manipulator);

    // Setup called in manipulator setup and creates/populates tabs by calling helpers
    void Setup();

    // Updates all existing readout tabs with current values
    void Update();

    TuningValues* GetTune() const;
    units::degree_t GetPivotAngle() const;
    units::revolutions_per_minute_t GetShooterRPM() const;
    double GetIntakeDrive() const;
    double GetDiverterDrive() const;
    double GetTrapDrive() const;
    units::inch_t GetArmTarget() const;
    units::inch_t GetWinchForwardLimit() const;
    units::inch_t GetWinchReverseLimit() const;

    void SetTuningShooterParameters(units::degree_t pivot_angle, units::revolutions_per_minute_t shooter_speed);

private:
    //==========================================================================
    // Private Nested Types


    frc::SimpleWidget* m_laser1_distance_widget = nullptr;
    frc::SimpleWidget* m_laser2_distance = nullptr;

    // Struct containing widgets contained in the intake layout
    struct IntakeWidgets {   
        frc::SimpleWidget* m_intake_current_widget;
        frc::SimpleWidget* m_intake_output_widget;
    };

    struct ShooterWidgets {
        frc::SimpleWidget* m_shooter_current_widget = nullptr;
        frc::SimpleWidget* m_shooter_rpm_widget = nullptr;
    };
    
    struct DiverterWidgets {
        frc::SimpleWidget* m_diverter_current_widget = nullptr;
        frc::SimpleWidget* m_diverter_output_widget = nullptr;
    };

    struct PivotWidgets {
        frc::SimpleWidget* m_pivot_current_widget = nullptr;
        frc::SimpleWidget* m_pivot_output_widget = nullptr;
        frc::SimpleWidget* m_pivot_angle_widget = nullptr;
        frc::SimpleWidget* m_pivot_bottom_limit_widget = nullptr;
        frc::SimpleWidget* m_pivot_top_limit_widget = nullptr;
    };
    
    struct TramplerWidgets {
        frc::SimpleWidget* m_trampler_current_widget;
        frc::SimpleWidget* m_trampler_output_widget;
    };

    struct WinchWidgets {
        frc::SimpleWidget* m_winch_current_widget;
        frc::SimpleWidget* m_winch_output_widget;
        frc::SimpleWidget* m_winch_position_widget;
        frc::SimpleWidget* m_forward_limit_widget;
        frc::SimpleWidget* m_reverse_limit_widget;
    };

    struct LiftWidgets {
        frc::SimpleWidget* m_lift_current_widget = nullptr;
        frc::SimpleWidget* m_lift_output_widget = nullptr;
        frc::SimpleWidget* m_lift_extension_widget = nullptr;
        frc::SimpleWidget* m_lift_bottom_limit_widget = nullptr;
        frc::SimpleWidget* m_lift_top_limit_widget = nullptr;
        frc::SimpleWidget* m_set_extension_widget;
    };

    struct TuningWidgets {
        frc::SimpleWidget* m_pivot_angle_widget;
        frc::SimpleWidget* m_shooter_rpm_widget;
        frc::SimpleWidget* m_intake_drive_widget;
        frc::SimpleWidget* m_diverter_drive_widget;
        frc::SimpleWidget* m_trap_drive_widget;
        frc::SimpleWidget* m_arm_target_widget;
    };
    // Helper methods that isolate setup for each mechanism/tuner
    // Sets up relevant widgets in a layout on a specified tab
    // 
    // tab - shuffleboard tab to add the layout to
    // x - x-coordinate of left edge of the layout on 'tab'
    // y - y-coordinate of top edge of the layout on 'tab'
    void SetupIntake(frc::ShuffleboardTab& tab, int x, int y);
    void SetupShooter(frc::ShuffleboardTab& tab, int x, int y);
    void SetupDiverter(frc::ShuffleboardTab& tab, int x, int y);
    void SetupPivot(frc::ShuffleboardTab& tab, int x, int y);
    void SetupTrampler(frc::ShuffleboardTab& tab, int x, int y);
    void SetupWinch(frc::ShuffleboardTab& tab, int x, int y);
    void SetupLift(frc::ShuffleboardTab& tab, int x, int y);
    void SetupTune(frc::ShuffleboardTab& tab, int x, int y);

    frc::SimpleWidget* m_target_angle_widget = nullptr;


    const Manipulator* m_manipulator;

    IntakeWidgets* m_intake_widgets = nullptr;
    ShooterWidgets* m_shooter_widgets = nullptr;
    DiverterWidgets* m_diverter_widgets = nullptr;
    PivotWidgets* m_pivot_widgets = nullptr;
    TramplerWidgets* m_trampler_widgets = nullptr;
    WinchWidgets* m_winch_widgets = nullptr;
    LiftWidgets* m_lift_widgets = nullptr;
    TuningWidgets* m_tune_widgets = nullptr;
};
