//==============================================================================
// ManipulatorShuffleboard.cpp
//==============================================================================

#include "ManipulatorShuffleboard.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Shooter.h"
#include "Mechanisms/Diverter.h"
#include "Mechanisms/Pivot.h"
#include "Mechanisms/Trampler.h"
#include "Mechanisms/Lift.h"
#include "Mechanisms/Winch.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

ManipulatorShuffleboard::ManipulatorShuffleboard(const Manipulator* manipulator) {
    m_manipulator = manipulator;
}

void ManipulatorShuffleboard::Setup() {
	// Create a new 'Manipulator' shuffleboard tab
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("Manipulator");
	// m_laser1_distance_widget = &shuffleboard_tab.Add("Laser Distance (mm)",0.0).WithPosition(0,8).WithSize(3,2);
	// m_laser2_distance = &shuffleboard_tab.Add("Laser2", 0.0).WithPosition(3,8).WithSize(3,2);
	// Call setup functions for individual layouts and place them on the manipulator tab
	SetupTune(shuffleboard_tab, 0, 0);
    SetupIntake(shuffleboard_tab, 12, 0);
	SetupShooter(shuffleboard_tab, 12, 3);
	SetupDiverter(shuffleboard_tab, 12, 6);
	SetupPivot(shuffleboard_tab, 12, 9);
	SetupTrampler(shuffleboard_tab, 0, 9);
	SetupWinch(shuffleboard_tab, 0, 6);
	SetupLift(shuffleboard_tab, 0, 3);
	
	m_target_angle_widget = &shuffleboard_tab.Add("Target Angle", 0.0).WithPosition(24,0).WithSize(3,2);
}

void ManipulatorShuffleboard::Update() {

	// Update intake widgets if they exist
	if (m_intake_widgets != nullptr) {
		const Intake* intake = m_manipulator->GetIntake();
		IntakeWidgets* iw = m_intake_widgets;
		iw->m_intake_current_widget->GetEntry()->SetDouble(intake->GetCurrent());
	}

	// Update pivot widgets if they exist
	if (m_shooter_widgets != nullptr) {
		ShooterWidgets* sw = m_shooter_widgets;
		const Shooter* shooter = m_manipulator->GetShooter();
		sw->m_shooter_current_widget->GetEntry()->SetDouble(shooter->GetCurrent().value());
		sw->m_shooter_rpm_widget->GetEntry()->SetDouble(shooter->GetSpeed().value());
	}

	if (m_diverter_widgets != nullptr) {
		DiverterWidgets* dw = m_diverter_widgets;
		const Diverter* diverter = m_manipulator->GetDiverter();
		dw->m_diverter_current_widget->GetEntry()->SetDouble(diverter->GetCurrent());
		dw->m_diverter_output_widget->GetEntry()->SetDouble(diverter->GetOutput());
	}

	// if (m_laser1_distance_widget != nullptr) {
	// 	m_laser1_distance_widget->GetEntry()->SetDouble(m_manipulator->GetLaser1());
	// }
	// if (m_laser2_distance != nullptr) {
	// 	m_laser2_distance->GetEntry()->SetDouble(m_manipulator->GetLaser2());
	// }

	PivotWidgets* pw = m_pivot_widgets;
	if (pw != nullptr) {
		const Pivot* pivot = m_manipulator->GetPivot();
		pw->m_pivot_current_widget->GetEntry()->SetDouble(pivot->GetCurrent().value());
		pw->m_pivot_output_widget->GetEntry()->SetDouble(pivot->GetOutput());
		pw->m_pivot_angle_widget->GetEntry()->SetDouble(pivot->GetAngle().value());
		pw->m_pivot_bottom_limit_widget->GetEntry()->SetBoolean(pivot->GetBottomLimitSwitch());
		pw->m_pivot_top_limit_widget->GetEntry()->SetBoolean(pivot->GetTopLimitSwitch());
	} 

	if (m_trampler_widgets != nullptr) {
		const Trampler* trampler = m_manipulator->GetTrampler();
		TramplerWidgets* tw = m_trampler_widgets;
		
		tw->m_trampler_current_widget->GetEntry()->SetDouble(trampler->GetCurrent());
		tw->m_trampler_output_widget->GetEntry()->SetDouble(trampler->GetOutput());
	}

	if (m_winch_widgets != nullptr) {
		const Winch* winch = m_manipulator->GetWinch();
		WinchWidgets* ww = m_winch_widgets;

		ww->m_winch_current_widget->GetEntry()->SetDouble(winch->GetCurrent());
		ww->m_winch_output_widget->GetEntry()->SetDouble(winch->GetOutput());
		ww->m_winch_position_widget->GetEntry()->SetDouble(winch->GetWinchPosition().value());

	}

	if (m_lift_widgets != nullptr) {
		const Lift* lift = m_manipulator->GetLift();
		LiftWidgets* lw = m_lift_widgets;

		lw->m_lift_current_widget->GetEntry()->SetDouble(lift->GetCurrent());
		lw->m_lift_output_widget->GetEntry()->SetDouble(lift->GetOutput());
		lw->m_lift_extension_widget->GetEntry()->SetDouble(lift->GetExtensionInch().value());
		lw->m_lift_bottom_limit_widget->GetEntry()->SetBoolean(lift->IsAtZeroLimitSwitch());
		lw->m_lift_top_limit_widget->GetEntry()->SetBoolean(lift->IsAtFullLimitSwitch());
		lw->m_set_extension_widget->GetEntry()->SetDouble(lift->GetSetExtension().value());
	}

	if (m_target_angle_widget != nullptr) {
		m_target_angle_widget->GetEntry()->SetDouble(m_manipulator->GetTargetAngle().value());
	}
}


ManipulatorShuffleboard::TuningValues* ManipulatorShuffleboard::GetTune() const {
	TuningValues* tune = new TuningValues; 
	TuningWidgets* tw = m_tune_widgets;
	tune->pivot_angle = units::degree_t(tw->m_pivot_angle_widget->GetEntry()->GetDouble(19.7));
	tune->shooter_rpm = units::revolutions_per_minute_t(tw->m_shooter_rpm_widget->GetEntry()->GetDouble(3000.0));
	tune->intake_drive = tw->m_intake_drive_widget->GetEntry()->GetDouble(0.5);
	tune->diverter_drive = tw->m_diverter_drive_widget->GetEntry()->GetDouble(0.5);
	tune->trap_drive = tw->m_trap_drive_widget->GetEntry()->GetDouble(0.5);
	tune->arm_target = units::inch_t(tw->m_arm_target_widget->GetEntry()->GetDouble(10.0));

	return tune;
}

units::degree_t ManipulatorShuffleboard::GetPivotAngle() const {
	return units::degree_t(m_tune_widgets->m_pivot_angle_widget->GetEntry()->GetDouble(19.7));
}

units::revolutions_per_minute_t ManipulatorShuffleboard::GetShooterRPM() const {
	return units::revolutions_per_minute_t(m_tune_widgets->m_shooter_rpm_widget->GetEntry()->GetDouble(3000.0));
}

double ManipulatorShuffleboard::GetIntakeDrive() const {
	return m_tune_widgets->m_intake_drive_widget->GetEntry()->GetDouble(0.5);
}

double ManipulatorShuffleboard::GetDiverterDrive() const {
	return m_tune_widgets->m_diverter_drive_widget->GetEntry()->GetDouble(0.5);
}

double ManipulatorShuffleboard::GetTrapDrive() const {
	return m_tune_widgets->m_trap_drive_widget->GetEntry()->GetDouble(0.5);
}

units::inch_t ManipulatorShuffleboard::GetArmTarget() const {
	return units::inch_t(m_tune_widgets->m_arm_target_widget->GetEntry()->GetDouble(10.0));
}

void ManipulatorShuffleboard::SetTuningShooterParameters(units::degree_t pivot_angle,
														 units::revolutions_per_minute_t shooter_speed) {
	m_tune_widgets->m_pivot_angle_widget->GetEntry()->SetDouble(pivot_angle.value());
	m_tune_widgets->m_shooter_rpm_widget->GetEntry()->SetDouble(shooter_speed.value());
}


void ManipulatorShuffleboard::SetupIntake(frc::ShuffleboardTab& tab, int x, int y) {
	// Create an 'Intake' layout on 'tab' with top-left at (x,y)
    frc::ShuffleboardLayout &layout =
        tab.GetLayout("Intake",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the IntakeWidgets struct
	m_intake_widgets = new IntakeWidgets;
	IntakeWidgets* iw = m_intake_widgets;

	// Populate the layout
    iw->m_intake_current_widget = &layout.Add("Roller Current", 0.0).WithPosition(0, 0).WithSize(3, 2);
}

void ManipulatorShuffleboard::SetupShooter(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Shooter' layout on 'tab' with top-left at (x,y)
    frc::ShuffleboardLayout &layout =
        tab.GetLayout("Shooter",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the ShooterWidgets struct
	m_shooter_widgets = new ShooterWidgets;
	ShooterWidgets* sw = m_shooter_widgets;

	// Populate the layout
	sw->m_shooter_current_widget = &layout.Add("Shooter Current", 0.0).WithPosition(0,0).WithSize(3,2);
	sw->m_shooter_rpm_widget = &layout.Add("Shooter RPM", 0.0).WithPosition(1,0).WithSize(3,2);
}

void ManipulatorShuffleboard::SetupDiverter(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Diverter' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Diverter",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the DiverterWidgets struct
	m_diverter_widgets = new DiverterWidgets;
	DiverterWidgets* dw = m_diverter_widgets;

	// Populate the layout
	dw->m_diverter_current_widget = &layout.Add("Diverter Current", 0.0).WithPosition(0,0).WithSize(3,2);
	dw->m_diverter_output_widget = &layout.Add("Diverter Output", 0.0).WithPosition(1,0).WithSize(3,2);
}

void ManipulatorShuffleboard::SetupPivot(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Pivot' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Pivot",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the PivotWidgets struct
	m_pivot_widgets = new PivotWidgets;
	PivotWidgets* pw = m_pivot_widgets;

	// Populate the layout
	pw->m_pivot_current_widget = &layout.Add("Current", 0.0).WithPosition(0,0).WithSize(2,2);
	pw->m_pivot_output_widget = &layout.Add("Output", 0.0).WithPosition(1,0).WithSize(2,2);
	pw->m_pivot_angle_widget = &layout.Add("Angle", 0.0).WithPosition(2,0).WithSize(2,2);
	pw->m_pivot_bottom_limit_widget = &layout.Add("Bottom Limit", false).WithPosition(3,0).WithSize(2,2);
	pw->m_pivot_top_limit_widget = &layout.Add("Top Limit", false).WithPosition(4,0).WithSize(2,2);
}

void ManipulatorShuffleboard::SetupTrampler(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Trapper' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Trapper",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the TrapperWidgets struct
	m_trampler_widgets = new TramplerWidgets;
	TramplerWidgets* tw = m_trampler_widgets;

	// Populate the layout
	tw->m_trampler_current_widget = &layout.Add("Current", 0.0).WithPosition(0,0).WithSize(2,2);
	tw->m_trampler_output_widget = &layout.Add("Output", 0.0).WithPosition(1,0).WithSize(2,2);

}

void ManipulatorShuffleboard::SetupWinch(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Winch' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Winch",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the WinchWidgets struct
	m_winch_widgets = new WinchWidgets;
	WinchWidgets* ww = m_winch_widgets;

	// Populate the layout
	ww->m_winch_current_widget = &layout.Add("Current", 0.0).WithPosition(0,0).WithSize(2,2);
	ww->m_winch_output_widget = &layout.Add("Output", 0.0).WithPosition(1,0).WithSize(2,2);
	ww->m_winch_position_widget = &layout.Add("Position", 0.0).WithPosition(2,0).WithSize(2,2);
	ww->m_forward_limit_widget = &layout.Add("Forward Limit", Winch::kForwardLimit.value()).WithPosition(3,0).WithSize(2,2);
	ww->m_reverse_limit_widget = &layout.Add("Reverse Limit", Winch::kReverseLimit.value()).WithPosition(4,0).WithSize(2,2);
}

void ManipulatorShuffleboard::SetupLift(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Lift' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Lift",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the LiftWidgets struct
	m_lift_widgets = new LiftWidgets;
	LiftWidgets* lw = m_lift_widgets;

	// Populate the layout
	lw->m_lift_current_widget = &layout.Add("Current", 0.0).WithPosition(0,0).WithSize(2,2);
	lw->m_lift_output_widget = &layout.Add("Output", 0.0).WithPosition(1,0).WithSize(2,2);
	lw->m_lift_extension_widget = &layout.Add("Extension", 0.0).WithPosition(2,0).WithSize(2,2);
	lw->m_lift_bottom_limit_widget = &layout.Add("Bottom Limit", false).WithPosition(3,0).WithSize(2,2);
	lw->m_lift_top_limit_widget = &layout.Add("Top Limit", false).WithPosition(4,0).WithSize(2,2);
	lw->m_set_extension_widget = &layout.Add("Set extension", 0.0).WithPosition(5,0).WithSize(2,2);
}

void ManipulatorShuffleboard::SetupTune(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Tune' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Tune",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the TuningWidgets struct
	m_tune_widgets = new TuningWidgets;
	TuningWidgets* tw = m_tune_widgets;

	// Populate the layout
	tw->m_pivot_angle_widget = &layout.Add("Pivot Set Angle", 63.0).WithPosition(0,0).WithSize(2,2);
	tw->m_shooter_rpm_widget = &layout.Add("Shooter Set RPM", 3000.0).WithPosition(1,0).WithSize(2,2);
	tw->m_intake_drive_widget = &layout.Add("Intake Drive", 0.5).WithPosition(2,0).WithSize(2,2);
	tw->m_diverter_drive_widget = &layout.Add("Diverter Drive",0.5).WithPosition(3,0).WithSize(2,2);
	tw->m_trap_drive_widget = &layout.Add("Trampler Drive", 0.5).WithPosition(4,0).WithSize(2,2);
	tw->m_arm_target_widget	= &layout.Add("Arm Target", 10.0).WithPosition(5,0).WithSize(2,2);
}

units::inch_t ManipulatorShuffleboard::GetWinchForwardLimit() const {
	return units::inch_t(m_winch_widgets->m_forward_limit_widget->GetEntry()->GetDouble(0.0));
}

units::inch_t ManipulatorShuffleboard::GetWinchReverseLimit() const {
	return units::inch_t(m_winch_widgets->m_reverse_limit_widget->GetEntry()->GetDouble(0.0));
}