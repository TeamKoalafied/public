#include "ManipulatorShuffleboard.h"
#include "Mechanisms/Arm.h"
#include "Mechanisms/NewIntake.h"
#include "Mechanisms/Pivot.h"
#include "Mechanisms/Wrist.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

ManipulatorShuffleboard::ManipulatorShuffleboard(const Manipulator* manipulator, const Arm* arm, const NewIntake* intake, 
                            const Pivot* pivot, const Wrist* wrist) {
    m_manipulator = manipulator;
	m_arm = arm;
	m_intake = intake;
	m_pivot = pivot;
	m_wrist = wrist;
}

void ManipulatorShuffleboard::Setup() {
	// Create a new 'Manipulator' shuffleboard tab
	std::cout << "ManipShuffle::Setup()\n";
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("Manipulator");

	// Call setup functions for individual layouts and place them on the manipulator tab
    SetupArm(shuffleboard_tab, 0, 3);
    SetupIntake(shuffleboard_tab, 12, 0);
    SetupPivot(shuffleboard_tab, 0, 0);
    SetupWrist(shuffleboard_tab, 12, 3);
	SetupTuneIntake(shuffleboard_tab, 12, 7);
	SetupTunePosition(shuffleboard_tab, 0, 6);
	std::cout << "Done Manip Shuffle setup\n";
}

void ManipulatorShuffleboard::Update() {
	// Update arm widgets if they exist
	if (m_arm_widgets != nullptr) {
		ArmWidgets* aw = m_arm_widgets;
		aw->m_current_widget->GetEntry()->SetDouble(m_arm->GetArmCurrent());
		aw->m_extension_widget->GetEntry()->SetDouble(m_arm->GetExtensionInch().value());
		aw->m_reverse_limit_widget->GetEntry()->SetBoolean(m_arm->IsAtZeroLimitSwitch());
		aw->m_forward_limit_widget->GetEntry()->SetBoolean(m_arm->IsAtFullLimitSwitch());
	}

	// Update intake widgets if they exist
	if (m_intake_widgets != nullptr) {
		IntakeWidgets* iw = m_intake_widgets;
		iw->m_roller_current_widget->GetEntry()->SetDouble(m_intake->GetCurrent());
	}

	// Update pivot widgets if they exist
	if (m_pivot_widgets != nullptr) {
		PivotWidgets* pw = m_pivot_widgets;
		pw->m_angle_widget->GetEntry()->SetDouble(m_pivot->GetPivotAngleDegrees().value());
		pw->m_current_widget->GetEntry()->SetDouble(m_pivot->GetPivotCurrent());
		pw->m_forward_limit_widget->GetEntry()->SetBoolean(m_pivot->IsAtZeroLimitSwitch());
		pw->m_reverse_limit_widget->GetEntry()->SetBoolean(m_pivot->IsAtFullLimitSwitch());
	}

	// Update wrist widgets if they exist
	if (m_wrist_widgets != nullptr) {
		WristWidgets* ww = m_wrist_widgets;
		ww->m_angle_widget->GetEntry()->SetDouble(m_wrist->GetWristAngleDegrees().value());
		ww->m_current_widget->GetEntry()->SetDouble(m_wrist->GetWristCurrent());
		ww->m_forward_limit_widget->GetEntry()->SetBoolean(m_wrist->IsAtZeroLimitSwitch());
		ww->m_reverse_limit_widget->GetEntry()->SetBoolean(m_wrist->IsAtFullLimitSwitch());
		ww->m_wrist_offset_widget->GetEntry()->SetDouble(m_manipulator->GetWristOffset().value());
	}
}

void ManipulatorShuffleboard::SetupArm(frc::ShuffleboardTab& tab, int x, int y) {
	// Create an 'Arm' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
	tab.GetLayout("Arm",frc::BuiltInLayouts::kGrid)
		.WithPosition(x, y)
		.WithSize(12, 2);

	// Initialise the ArmWidgets struct
	m_arm_widgets = new ArmWidgets;
	ArmWidgets* aw = m_arm_widgets;

	// Add the widgets to the layout
	aw->m_forward_limit_widget = &layout.Add("Forward limit", false)
								.WithPosition(0,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);
	aw->m_reverse_limit_widget = &layout.Add("Reverse limit", false)
								.WithPosition(1,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);

    aw->m_current_widget = &layout.Add("Arm Current", 0.0).WithPosition(2, 0).WithSize(3, 2);
	aw->m_extension_widget = &layout.Add("Arm extension",0.0).WithPosition(3,0).WithSize(3,2);
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

	// Add the widgets to the layout
    iw->m_roller_current_widget = &layout.Add("Roller Current", 0.0).WithPosition(4, 0).WithSize(3, 2);
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

	// Add the widgets to the layout
	pw->m_forward_limit_widget = &layout.Add("Forward limit", false)
								.WithPosition(0,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);
	pw->m_reverse_limit_widget = &layout.Add("Reverse limit", false)
								.WithPosition(1,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);

    pw->m_current_widget = &layout.Add("Pivot Current", 0.0).WithPosition(2, 0).WithSize(3, 2);
	pw->m_angle_widget = &layout.Add("Pivot Angle",0.0).WithPosition(3,0).WithSize(3,2);
}

void ManipulatorShuffleboard::SetupWrist(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Wrist' layout on 'tab' with top-left at (x,y)
    frc::ShuffleboardLayout &layout =
        tab.GetLayout("Wrist",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	// Initialise the WristWidgets struct
	m_wrist_widgets = new WristWidgets;
	WristWidgets* ww = m_wrist_widgets;

	// Add the widgets to the layout
    ww->m_wrist_offset_widget = &layout.Add("Wrist Offset", 0.0).WithPosition(0, 0).WithSize(3, 2);
    ww->m_forward_limit_widget = &layout.Add("Forward limit", false)
								.WithPosition(0,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);
	ww->m_reverse_limit_widget = &layout.Add("Reverse limit", false)
								.WithPosition(1,0)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);

    ww->m_current_widget = &layout.Add("Wrist Current", 0.0).WithPosition(2, 0).WithSize(3, 2);
	ww->m_angle_widget = &layout.Add("Wrist Angle",0.0).WithPosition(3,0).WithSize(3,2);
}


void ManipulatorShuffleboard::SetupTuneIntake(frc::ShuffleboardTab& tab, int x, int y) {
	// Create an 'Intake Modifiable' layout on 'tab' with top-left at (x,y)
    frc::ShuffleboardLayout &layout =
        tab.GetLayout("Intake Modifiable",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(9, 4);

	// Initialise the IntakeTuningWidgets struct
    m_intake_tune_widgets = new IntakeTuningWidgets;
    IntakeTuningWidgets* itw = m_intake_tune_widgets;

	// Add the widgets to the layout
    itw->m_cone_speed = &layout.Add("Cone speed", DEFAULT_CONE_SPEED).WithPosition(0,0).WithSize(3,2);
    itw->m_cube_speed = &layout.Add("Cube speed", DEFAULT_CUBE_SPEED).WithPosition(0,2).WithSize(3,2);
    itw->m_cone_current = &layout.Add("Cone current", DEFAULT_CONE_CURRENT).WithPosition(3,0).WithSize(3,2);
    itw->m_cube_current = &layout.Add("Cube current", DEFAULT_CUBE_CURRENT).WithPosition(3,2).WithSize(3,2);  
    itw->m_cone_timeout = &layout.Add("Cone timeout", DEFAULT_CONE_TIMEOUT).WithPosition(6,0).WithSize(3,2);
    itw->m_cube_timeout = &layout.Add("Cube timeout", DEFAULT_CUBE_TIMEOUT).WithPosition(6,2).WithSize(3,2); 
}

void ManipulatorShuffleboard::SetupTunePosition(frc::ShuffleboardTab& tab, int x, int y) {
	// Create a 'Test Position' layout on 'tab' with top-left at (x,y)
	frc::ShuffleboardLayout &layout =
        tab.GetLayout("Test Position",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);
	
	// Initialise the TunePositionWidgets struct
    m_tune_position_widgets = new TunePositionWidgets;
	TunePositionWidgets* tpw = m_tune_position_widgets;

	// Add the widgets to the layout
    tpw->m_pivot_angle_widget = &layout.Add("Pivot Angle", 0.0).WithPosition(0, 0).WithSize(3, 2);
    tpw->m_arm_extension_widget = &layout.Add("Arm Extension", 0.0).WithPosition(1, 0).WithSize(3, 2);
    tpw->m_wrist_angle_widget = &layout.Add("Wrist Angle", 0.0).WithPosition(2, 0).WithSize(3, 2);
}

ManipulatorShuffleboard::IntakeParams* ManipulatorShuffleboard::GetIntakeTune() {
	// Create struct to return
	IntakeParams* intake_tune = new IntakeParams;
	
	// Return default values if the IntakeTuneWidgets have not been initialised
	if (m_intake_tune_widgets == nullptr) {
		intake_tune->cone_current = DEFAULT_CONE_CURRENT;
		intake_tune->cone_speed = DEFAULT_CONE_SPEED;
		intake_tune->cone_timeout = DEFAULT_CONE_TIMEOUT;
		intake_tune->cube_current = DEFAULT_CUBE_CURRENT;
		intake_tune->cube_speed = DEFAULT_CUBE_SPEED;
		intake_tune->cube_timeout = DEFAULT_CUBE_TIMEOUT;
		return intake_tune;
	}	

	// Copy current values from the shuffleboard into the struct
	IntakeTuningWidgets* itw = m_intake_tune_widgets;

	intake_tune->cone_current = itw->m_cone_current->GetEntry()->GetDouble(DEFAULT_CONE_CURRENT);
	intake_tune->cone_speed = itw->m_cone_speed->GetEntry()->GetDouble(DEFAULT_CONE_SPEED);
	intake_tune->cone_timeout = itw->m_cone_timeout->GetEntry()->GetDouble(DEFAULT_CONE_TIMEOUT);
	intake_tune->cube_current = itw->m_cube_current->GetEntry()->GetDouble(DEFAULT_CUBE_CURRENT);
	intake_tune->cube_speed = itw->m_cube_speed->GetEntry()->GetDouble(DEFAULT_CUBE_SPEED);
	intake_tune->cube_timeout = itw->m_cube_timeout->GetEntry()->GetDouble(DEFAULT_CUBE_TIMEOUT);

	return intake_tune;
}

ManipulatorShuffleboard::PositionParams* ManipulatorShuffleboard::GetCustomPosition() {
	// Return nothing if the TunePositionWidgets have not been initialised
	if (m_tune_position_widgets != nullptr) return nullptr;

	// Copy the custom position values into a PositionParams struct
	TunePositionWidgets* tpw = m_tune_position_widgets;
	PositionParams* pos_tune = new PositionParams;

	pos_tune->pivot_angle = units::degree_t(tpw->m_pivot_angle_widget->GetEntry()->GetDouble(0.0));
	pos_tune->wrist_angle = units::degree_t(tpw->m_wrist_angle_widget->GetEntry()->GetDouble(0.0));
	pos_tune->arm_extension = units::inch_t(tpw->m_arm_extension_widget->GetEntry()->GetDouble(0.0));

	return pos_tune;
}

void ManipulatorShuffleboard::UpdateCustomPosition(PositionParams pos) {
	// Do nothing if TunePositionWidgets are uninitialised
	if (m_tune_position_widgets == nullptr) return;

	// Update custom position values
	TunePositionWidgets* tpw = m_tune_position_widgets;
	tpw->m_arm_extension_widget->GetEntry()->SetDouble(pos.arm_extension.value());
	tpw->m_pivot_angle_widget->GetEntry()->SetDouble(pos.pivot_angle.value());
	tpw->m_arm_extension_widget->GetEntry()->SetDouble(pos.wrist_angle.value());
}