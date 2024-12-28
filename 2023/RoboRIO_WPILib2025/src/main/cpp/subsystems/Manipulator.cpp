//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "SwerveDrivebase.h"
#include "ManipulatorShuffleboard.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Pivot.h"
#include "Mechanisms/Arm.h"
#include "Mechanisms/Wrist.h"
#include "Mechanisms/NewIntake.h"

#include "../RobotConfiguration.h"
#include "../util/KoalafiedUtilities.h"
#include "../util/Logging.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Manipulator::Manipulator() :
    m_joystick_timer { "M Joystick", false },
    m_simulation_timer { "M Simulation", false },
    m_periodic_timer { "M Periodic", false }
 {

    //m_intake = new Intake();
    m_intake = new NewIntake;
    m_pivot = new Pivot;
    m_arm = new Arm;
    m_wrist = new Wrist;
    m_shuffleboard = new ManipulatorShuffleboard(this, m_arm, m_intake, m_pivot, m_wrist);

    // m_find_target_control = NULL;
    m_haptic_controller = NULL;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    m_periodic_timer.Start();

    // Perform the update for each of the mechanisms
    m_intake->Periodic();
    m_pivot->Periodic();
    m_arm->Periodic();
    m_wrist->Periodic();
    m_shuffleboard->Update();
    
    // Update the haptic feedback
    m_haptic_controller->Periodic();

    m_periodic_timer.End();
}


//==============================================================================
// Setup and Shutdown

void Manipulator::Setup() {
    std::cout << "Manipulator::Setup()\n";

    m_controller = new frc::XboxController(RC::kJoystickPortOperator);
    m_haptic_controller = new HapticController(m_controller);



    // Setup all the mechanisms
    m_intake->Setup();
    m_arm->Setup();
    m_pivot->Setup();
    m_wrist->Setup();

    m_shuffleboard->Setup();

    SetDefaultCommand(frc2::RunCommand(
        [this] {
        DoJoystickControl();
        }, { this }));
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // TODO Do we need any shutdown functions

    // Shutdown all the mechanisms
    // m_shooter->Shutdown();
    // m_hood->Shutdown();
    m_intake->Shutdown();
    // m_indexer->Shutdown();
    // m_turret->Shutdown();
    //m_distance_sensor->Shutdown();

    StopLoggingGotoPosition(true);
}

void Manipulator::TeleopInit() {
    // Reset the manual position to unknown so that we do not start seeking a position
    // when the robot is disabled and reinitialise. This prevents the manipulator
    // driving wildly when the robot is reinitialised during testing and practice.
    m_manual_position = Position::Unknown;

    // Reset to cone mode
    m_game_piece = GamePiece::Cone;
}

void Manipulator::DisabledInit() {

}


//==============================================================================
// Manipulator Control


void Manipulator::GoToPosition(Position position) {
    const PositionInfo& position_info = GetPositionInfo(position);

    const double PIVOT_VELOCITY_FACTOR = 1.0 ;
    const double ARM_VELOCITY_FACTOR = 1.0;
    const double WRIST_VELOCITY_FACTOR = 1.0;

    // Calculate a slowdown for the pivot when the arm has to move a long distance
    double pivot_slowdown = 1.0;
    double arm_delta_inch = fabs((m_arm->GetExtensionInch() - position_info.m_arm_extension).value());
    if (arm_delta_inch > 15.0) {
        pivot_slowdown = 1.0 - 0.5 * (arm_delta_inch - 15.0)/15.0;
    }

    m_pivot->DriveToAngleDegrees(position_info.m_pivot_angle, PIVOT_VELOCITY_FACTOR * pivot_slowdown);
    m_arm->SetExtensionInch(position_info.m_arm_extension, ARM_VELOCITY_FACTOR);
    // m_wrist->DriveToAngleDegrees(position_info.m_wrist_angle, WRIST_VELOCITY_FACTOR);
}

void Manipulator::DropGamePiece() {
    switch (m_game_piece) {
        case GamePiece::Cone: m_intake->DropCone(); break;
        case GamePiece::Cube: m_intake->DropCube(); break;
    }
}

void Manipulator::StopIntake() {
    m_intake->Stop();
}


//==============================================================================
// Manipulator State

bool Manipulator::IsAtPosition(Position position, bool low_tolerance) {
    // Get the angles and extension for the desired postition
    const PositionInfo& position_info = GetPositionInfo(position);

    // If seeking to 0 position for any mechanism then we must hit the limit switch
    if (position_info.m_pivot_angle == 0_deg && !m_pivot->IsAtZeroLimitSwitch()) {
        return false;
    }
    if (position_info.m_arm_extension == 0_in && !m_arm->IsAtZeroLimitSwitch()) {
        return false;
    }
    // if (position_info.m_wrist_angle == 0_deg && !m_wrist->IsAtZeroLimitSwitch()) {
    //     return false;
    // }

    // Calculate the distance that each mechanism is from the desired position
    units::inch_t arm_error = units::inch_t(fabs((m_arm->GetExtensionInch() - position_info.m_arm_extension).value()));
    units::degree_t pivot_error = units::degree_t(fabs((m_pivot->GetPivotAngleDegrees() - position_info.m_pivot_angle).value()));
    units::degree_t wrist_error = units::degree_t(fabs((m_wrist->GetWristAngleDegrees() - position_info.m_wrist_angle).value()));

    // We are at the position if the errors are with a tolerance
    const units::degree_t PIVOT_TOLERANCE = low_tolerance ? 2.0_deg : 0.5_deg;
    const units::inch_t ARM_TOLERANCE = 0.5_in;
    const units::degree_t WRIST_TOLERANCE = low_tolerance ? 2.0_deg : 1.0_deg;
    return pivot_error < PIVOT_TOLERANCE &&
           arm_error < ARM_TOLERANCE; //&&
        //    wrist_error < WRIST_TOLERANCE;
}

units::inch_t Manipulator::GetArmExtension() {
    return m_arm->GetExtensionInch();
}

units::degree_t Manipulator::GetWristOffset() const {
    return m_wrist_manual_offset;
}

//==========================================================================
// Joystick Control

void Manipulator::DoJoystickControl() {

    // IMPORTANT: Only one of the following lines should ever be uncommented at a time.
    // The TestDrive*() functions are only used during development and testing.
    DoManualJoystickControl(m_controller);
//    m_intake->TestDriveIntake(m_controller);
//    m_pivot->TestDrivePivot(m_controller);
//    m_wrist->TestDriveWrist(m_controller);
//    m_arm->TestDriveArm(m_controller);
}

void Manipulator::DoManualJoystickControl(frc::XboxController* joystick) {
    if (joystick->GetLeftBumper()) {
        if (joystick->GetRightBumper()) {
            ManipulatorShuffleboard::PositionParams pos;
            pos.arm_extension = m_arm->GetExtensionInch();
            pos.pivot_angle = m_pivot->GetPivotAngleDegrees();
            pos.wrist_angle = m_wrist->GetWristAngleDegrees();
            m_shuffleboard->UpdateCustomPosition(pos);
            return;
        }
    }

    m_joystick_timer.Start();

    // Determine it the POV, or other position button, is being held, which indicates to move
    // to a given position for the pivot/arm/wrist
    Position manual_position = Position::Unknown;
    switch (m_pov_filter.Filter(joystick->GetPOV())) {
        case RC::kJoystickPovUp:
            manual_position = m_game_piece == GamePiece::Cone ? Position::Level3ConePlace
                                                              : Position::Level3CubePlace;
            break;
        case RC::kJoystickPovLeft:
            manual_position = Position::Stowed;
            break;
        case RC::kJoystickPovDown:
            manual_position = m_game_piece == GamePiece::Cone ? Position::GroundConePickup
                                                              : Position::GroundCubePickup;
            break;
        case RC::kJoystickPovRight:
            manual_position = m_game_piece == GamePiece::Cone ? Position::ShelfConePickup
                                                              : Position::ShelfCubePickup;
            break;
        default:
            if (joystick->GetBackButton()) {
                manual_position = m_game_piece == GamePiece::Cone ? Position::Level2ConePlace
                                                                  : Position::Level2CubePlace;
            } else if (joystick->GetStartButton()) {
                manual_position = Position::Starting;
            } else if (joystick->GetRightBumper()) {
                manual_position = Position::TestCustom;
            }
            break;
    }

    units::degree_t pivot_angle_degrees = m_pivot->GetPivotAngleDegrees();
    if (manual_position != Position::Unknown) {
        // User wants to do to a set position so check if we are already doing that
        if (manual_position != m_manual_position) {
            // New set position so record it and start going to it
            m_manual_position = manual_position;
            m_manual_position_complete = false;
            m_at_position_lower_tolerance = false;
            m_position_timer.Stop();
            m_position_timer.Reset();
            GoToPosition(m_manual_position);
            StartLoggingGotoPosition();
        } else if (!m_manual_position_complete) {
            UpdateLoggingGotoPosition();
            GoToPosition(m_manual_position);

            // Check if we are at the desired position (first with high tolerance)
            bool at_position = IsAtPosition(m_manual_position, false);
            if (!at_position) {
                // If not at the position with high tolerance, check a lower tolerance and if we maintain
                // the position at low tolerane for 1_s just regard the movement to the position as done.
                if (IsAtPosition(m_manual_position, true)) {
                    if (m_at_position_lower_tolerance) {
                        if (m_position_timer.HasElapsed(1_s)) {
                            at_position = true;
                        }
                    } else {
                        m_at_position_lower_tolerance = true;
                        m_position_timer.Start();
                        m_position_timer.Reset();
                    }
                } else {
                    m_at_position_lower_tolerance = false;
                    m_position_timer.Stop();
                    m_position_timer.Reset();
                }
            }

            // We have reached the position so do a haptic buzz and record that we are finished
            if (at_position) {
                m_haptic_controller->DoContinuousFeedback(0.5, 1.0);
                m_manual_position_complete = true;
                StopLoggingGotoPosition(false);
            }
        } else {
            // We are already at the position so do nothing
        }

        // Update the wrist offset so that if we switch to manual control the
        // wrist does not start moving to the 'virtual 4-bar' position
        units::degree_t wrist_angle_degrees = m_wrist->GetWristAngleDegrees();
        m_wrist_manual_offset = wrist_angle_degrees - GetWristAngleFromPivot(pivot_angle_degrees);

    } else {

        // Not going to a set position so record that and then do manual driving
        if (m_manual_position != Position::Unknown) {
            m_manual_position = Position::Unknown;
            m_manual_position_complete = false;
            StopLoggingGotoPosition(true);
        }

        // Determine a pivot drive speed. Use a small percentage of full power and
        // slow it down if the arm is extended as it is dangerously fast.
        const double RETRACTED_PIVOT_SPEED = 0.2;
        const double EXTENDED_PIVOT_SPEED = 0.05;
        double extension_fraction = m_arm->GetExtensionInch() / RC::kArmMaximumExtensionInch;
        double pivot_max_speed = (1.0 - extension_fraction) * RETRACTED_PIVOT_SPEED +
                                 extension_fraction * EXTENDED_PIVOT_SPEED;

        // Drive the pivot with the Y axis of the right stick. Use a small percentage
        // of full power as it is dangerously fast.
        double pivot_drive = frc::ApplyDeadband(-m_controller->GetRightY(), RC::kJoystickDeadzone);
        pivot_drive = KoalafiedUtilities::PowerAdjust(pivot_drive, 2.0);
        pivot_drive *= pivot_max_speed;
        if (pivot_drive == 0) {
            // If the user stops driving the pivot, lock it at its current position
            if(!m_pivot->IsAngleSet()) {
                m_pivot->DriveToAngleDegrees(pivot_angle_degrees, 1.0);
            }
        } else {
            m_pivot->ManualDrivePivot(pivot_drive);
        }
/*

        // Use the left Y axis to adjust the wrist offset. Clip it so that it never tries
        // to put the wrist beyond its valid range (the wrist position would be clipped
        // but the operator could otherwise need to wait to 'unwind' the value before
        // anything would move)
        const units::degree_t WRIST_MAX_SPEED_PER_PERIOD = 1.0_deg;
        double wrist_drive = frc::ApplyDeadband(m_controller->GetLeftY(), RC::kJoystickDeadzone);
        m_wrist_manual_offset += wrist_drive * WRIST_MAX_SPEED_PER_PERIOD;
        units::degree_t wrist_angle_from_pivot = GetWristAngleFromPivot(pivot_angle_degrees);
        units::degree_t max_wrist_offset = Wrist::kWristMaximumForwardDegrees - wrist_angle_from_pivot + 45_deg;
        units::degree_t min_wrist_offset = Wrist::kWristMaximumReverseDegrees - wrist_angle_from_pivot - 45_deg;
        if (m_wrist_manual_offset > max_wrist_offset) m_wrist_manual_offset = max_wrist_offset;
        if (m_wrist_manual_offset < min_wrist_offset) m_wrist_manual_offset = min_wrist_offset;

        // Set the wrist position from the pivot, plus any manaul offset
        units::degree_t wrist_angle_degrees = wrist_angle_from_pivot + m_wrist_manual_offset;
        // m_wrist->DriveToAngleDegrees(wrist_angle_degrees, 1.0);

        // Drive the arm using the triggers. Right is out and left is back. When the arm is
        // close to the ends slow it down so it does not slam into the ends.
        double arm_drive = 0;
        double left_bumper = m_controller->GetLeftTriggerAxis();
        double right_bumper = m_controller->GetRightTriggerAxis();
        if (left_bumper > 0 && right_bumper == 0) {
            arm_drive = -left_bumper;
            if (m_arm->GetExtensionInch() < 5.0_in) {
                arm_drive *= 0.5;
            }
        }
        if (left_bumper == 0 && right_bumper > 0) {
            arm_drive = right_bumper;
            if (m_arm->GetExtensionInch() > 25.0_in) {
                arm_drive *= 0.5;
            }
        }

        // Drive the arm at full speed
        arm_drive *= 1.0;
        m_arm->ManualDriveArm(arm_drive);
*/
    }

    // THe A button grabs a game piece and the Y button drops it. If neither we just stop the intake.
    bool now_intaking = false;
    if (joystick->GetAButton()) {
        ManipulatorShuffleboard::IntakeParams* intake_tune = m_shuffleboard->GetIntakeTune();
        switch (m_game_piece) {
            case GamePiece::Cone: m_intake->IntakeCone(intake_tune->cone_current, 
                                                        intake_tune->cone_speed, 
                                                        intake_tune->cone_timeout); 
            break;
            case GamePiece::Cube: m_intake->IntakeCube(intake_tune->cube_current, 
                                                        intake_tune->cube_speed,
                                                        intake_tune->cube_timeout); 
            break;
        }
        now_intaking = true;
    } else if (joystick->GetYButton()) {
        m_has_game_piece = false;
        switch (m_game_piece) {
            case GamePiece::Cone: m_intake->DropCone(); break;
            case GamePiece::Cube: m_intake->DropCube(); break;
        }
    } else {
        m_intake->Stop();
    }

    // Update intake logging and whether we have detected grabbing a game piece
    if (now_intaking) {
        if (!m_intaking) {
            // Intaking has just started so start logging, update the state and start a timer
            m_intaking = true;
            m_intake->StartLoggingIntake(GamePieceToString(m_game_piece));
            m_has_game_piece = false;
            m_intake_timer.Reset();
            m_intake_timer.Start();
            m_intake_initial_arm_extension = m_arm->GetExtensionInch();
        } else {
            // Intaking is continuing, so update logging and check if the intake has detected a game piece
            // (unless we already detected one).
            m_intake->UpdateLoggingIntake();
            if (m_intake->HasGamePiece() && !m_has_game_piece) {
                // We have a game piece so record that and let the operator know with haptic feedback
                m_has_game_piece = true;
                m_haptic_controller->DoContinuousFeedback(0.5, 1.0);
            }

            // If we are trying to intake a cone and have not grabbed a game piece after 1s then retract the arm
            // slowly (1" per second to a maximum of 2" or retraction). This hopefully moves the roller over the
            // point of the cone so it will catch. (Not sure if this ever worked well) 
            if (manual_position == Position::Unknown && !m_has_game_piece && m_intake_initial_arm_extension > 3_in &&
                m_game_piece == GamePiece::Cone) {
                units::inch_t arm_extension = m_intake_initial_arm_extension;
                if (m_intake_timer.Get() > 1_s) {
                    double extra_time = m_intake_timer.Get().value() - 1;
                    if (extra_time > 2) extra_time = 2;
                    arm_extension -= 1_in * extra_time;
                }
                m_arm->SetExtensionInch(arm_extension, 1.0);
            }
        }
    } else {
        // Not intaking at the moment. If we have just stopped, stop logging.
        if (m_intaking) {
            m_intaking = false;
            m_intake->StopLoggingIntake();
        }
    }

    // Use the X and B buttons to switch to Cones and Cubes respectively
    if (joystick->GetXButton()) {
        m_game_piece = GamePiece::Cone;
    } else if (joystick->GetBButton()) {
        m_game_piece = GamePiece::Cube;
    }

    m_joystick_timer.End();
}

units::degree_t Manipulator::GetWristAngleFromPivot(units::degree_t pivot_angle) {
    //
    units::degree_t wrist_angle_degrees = pivot_angle;
    if (wrist_angle_degrees > -10_deg) {
        wrist_angle_degrees *= 15_deg/10_deg;
    } else {
        wrist_angle_degrees -= 5_deg;
    }
    return wrist_angle_degrees;
}

void Manipulator::StartLoggingGotoPosition() {
    // Disable goto position logging
    // return;

    // Stop the previous log file if necessary
    StopLoggingGotoPosition(true);

    // Open the CSV log file
    const char* const RESULT_FILENAME = "/home/lvuser/logging/ManipulatorGotoPosition.csv";
    m_goto_position_csv_log_file = new Logging::CsvFile();
    m_goto_position_csv_log_file->Open(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_goto_position_csv_log_file->Fail()) {
        // If opening the file fails stop logging
        std::cout << "Manipulator::StartLoggingGotoPosition() - Failed to open log file\n";
        StopLoggingGotoPosition(true);
        return;
    }

    // Write an initial line saying which position we are going to
    *m_goto_position_csv_log_file << std::string("Starting at Position - ") + PositionToString(GetCurrentPosition()) << "\n";
    *m_goto_position_csv_log_file << std::string("Going to Position - ") + PositionToString(m_manual_position) << "\n";

    // Set the precision for numbers
    m_goto_position_csv_log_file->SetPrecision(3, true);

    // Write column heading for the data we are going to write
    *m_goto_position_csv_log_file << "Time";
    *m_goto_position_csv_log_file << "Pivot Angle" << "Pivot Drive" << "Pivot Current";
    *m_goto_position_csv_log_file << "Arm Ext" << "Arm Drive" << "Arm Current";
    *m_goto_position_csv_log_file << "Wrist Angle" << "Wrist Drive" << "Wrist Current" << "\n";

    // Start the timer and reset the record of the mechanisms getting to thier final positions
    m_goto_position_timer.Reset();
    m_goto_position_timer.Start();
    m_goto_position_pivot_time = 0.0;
    m_goto_position_arm_time = 0.0;
    m_goto_position_wrist_time = 0.0;
}

void Manipulator::UpdateLoggingGotoPosition() {
    // Do nothing if there is no file
    if (m_goto_position_csv_log_file == nullptr) return;

    // Get the position, output drive and current for the pivot, arm and wrist
    double pivot_angle = m_pivot->GetPivotAngleDegrees().value();
    double pivot_drive = m_pivot->GetPivotDrive();
    double pivot_current = m_pivot->GetPivotCurrent();

    double arm_ext = m_arm->GetExtensionInch().value();
    double arm_drive = m_arm->GetArmDrive();
    double arm_current = m_arm->GetArmCurrent();

    double wrist_angle = m_wrist->GetWristAngleDegrees().value();
    double wrist_drive = m_wrist->GetWristDrive();
    double wrist_current = m_wrist->GetWristCurrent();

    // Write the data to the file for the tieme and the position, output drive and current
    // for the pivot, arm and wrist
    m_goto_position_csv_log_file->SetPrecision(3, true);
    *m_goto_position_csv_log_file << m_goto_position_timer.Get().value();
    *m_goto_position_csv_log_file << pivot_angle << pivot_drive << pivot_current;
    *m_goto_position_csv_log_file << arm_ext << arm_drive << arm_current;
    *m_goto_position_csv_log_file << wrist_angle << wrist_drive << wrist_current << "\n";

    // Calculate the current absolute error for each mechanism
    const PositionInfo& position_info = GetPositionInfo(m_manual_position);
    units::inch_t arm_error = units::inch_t(fabs((m_arm->GetExtensionInch() - position_info.m_arm_extension).value()));
    units::degree_t pivot_error = units::degree_t(fabs((m_pivot->GetPivotAngleDegrees() - position_info.m_pivot_angle).value()));
    units::degree_t wrist_error = units::degree_t(fabs((m_wrist->GetWristAngleDegrees() - position_info.m_wrist_angle).value()));

    // If any mechanism has reached it final position (within the standard tolerance)
    // record the time it took
    const units::degree_t PIVOT_TOLERANCE = 0.5_deg;
    const units::inch_t ARM_TOLERANCE = 0.5_in;
    const units::degree_t WRIST_TOLERANCE = 1.0_deg;
    if  (pivot_error < PIVOT_TOLERANCE && m_goto_position_pivot_time == 0.0) {
        m_goto_position_pivot_time = m_goto_position_timer.Get().value();
        std::cout << "Pivot complete in " << m_goto_position_pivot_time << "s\n";
    }
    if  (arm_error < ARM_TOLERANCE && m_goto_position_arm_time == 0.0) {
        m_goto_position_arm_time = m_goto_position_timer.Get().value();
        std::cout << "Arm complete in " << m_goto_position_arm_time << "s\n";
    }
    if  (wrist_error < WRIST_TOLERANCE && m_goto_position_wrist_time == 0.0) {
        m_goto_position_wrist_time = m_goto_position_timer.Get().value();
        std::cout << "Wrist complete in " << m_goto_position_wrist_time << "s\n";
    }
}

void Manipulator::StopLoggingGotoPosition(bool cancelled) {
    if (m_goto_position_csv_log_file != nullptr) {
        // Record if the movement was cancelled
        if (cancelled) {
            *m_goto_position_csv_log_file << "CANCELLED" << "\n";
        }

        // Write the target angles and extension for the position we are going to. Align them
        // with the recorded data above.
        const PositionInfo& position_info = GetPositionInfo(m_manual_position);
        *m_goto_position_csv_log_file << "Target" << position_info.m_pivot_angle.value() << "" << "";
        *m_goto_position_csv_log_file << position_info.m_arm_extension.value() << "" << "";
        *m_goto_position_csv_log_file << position_info.m_wrist_angle.value() << "" << "" << "\n";

        // Log the time that each mechanism took to get the to right position
        *m_goto_position_csv_log_file << "Pivot" << m_goto_position_pivot_time << "\n";
        *m_goto_position_csv_log_file << "Arm" << m_goto_position_arm_time << "\n";
        *m_goto_position_csv_log_file << "Wrist" << m_goto_position_wrist_time << "\n";

        // Log that target and actual to the console for quick debugging of accuracy
        double pivot_angle = m_pivot->GetPivotAngleDegrees().value();
        double arm_ext = m_arm->GetExtensionInch().value();
        double wrist_angle = m_wrist->GetWristAngleDegrees().value();
        std::cout << "Pivot Target: " << position_info.m_pivot_angle.value() << "  Actual: " << pivot_angle << "\n";
        std::cout << "Arm Target: " << position_info.m_arm_extension.value() << "  Actual: " << arm_ext << "\n";
        std::cout << "Wrist Target: " << position_info.m_wrist_angle.value() << "  Actual: " << wrist_angle << "\n";
    }

    // Delete the log file
    delete m_goto_position_csv_log_file;
    m_goto_position_csv_log_file = nullptr;
}

constexpr const char* Manipulator::PositionToString(Position position)
{
    switch (position) {
        case Position::Unknown:          return "Unknown";
        case Position::TestCustom:       return "TestCustom";
        case Position::Starting:         return "Starting";
        case Position::Stowed:           return "Stowed";
        case Position::GroundConePickup: return "GroundConePickup";
        case Position::GroundCubePickup: return "GroundCubePickup";
        case Position::ShelfConePickup:  return "ShelfConePickup";
        case Position::ShelfCubePickup:  return "ShelfCubePickup";
        case Position::Level1ConePlace:  return "Level1ConePlace";
        case Position::Level1CubePlace:  return "Level1CubePlace";
        case Position::Level2ConePlace:  return "Level2ConePlace";
        case Position::Level2CubePlace:  return "Level2CubePlace";
        case Position::Level3ConePlace:  return "Level3ConePlace";
        case Position::Level3CubePlace:  return "Level3CubePlace";
    }
    return "<ERROR>";
}

constexpr const char* Manipulator::GamePieceToString(GamePiece gamepiece) {
    switch (gamepiece) {
        case GamePiece::Cone: return "Cone";
        case GamePiece::Cube: return "Cube";
    }
    return "<ERROR>";
}


//==============================================================================
// Position Control

const Manipulator::PositionInfo& Manipulator::GetPositionInfo(Position position) {
    if (position == Position::TestCustom) return GetCustomPositionInfo();

    static PositionInfo info[] = { 
        {Position::Starting,            0_deg,  0_in,    0_deg},
        {Position::Stowed,              0_deg,  0_in,    0_deg},

        {Position::GroundConePickup,  -88_deg,  0_in,  -96_deg},
        {Position::GroundCubePickup,  -88_deg,  0_in, -104_deg},

        {Position::ShelfConePickup,   -26_deg, 13.5_in,  -28_deg},
        {Position::ShelfCubePickup,   -28_deg,  4_in,  -55_deg},

        {Position::Level1ConePlace,   -88_deg,  0_in, -100_deg},
        {Position::Level1CubePlace,   -88_deg,  0_in, -100_deg},

        {Position::Level2ConePlace,   -40_deg,  9_in,  -41_deg},
        {Position::Level2CubePlace,   -52_deg,  8_in,  -58_deg},

        {Position::Level3ConePlace,   -43_deg, 24_in,  -64_deg},
        {Position::Level3CubePlace,   -53_deg, 27_in,  -67_deg},
        {Position::Unknown, 0_deg, 0_in, 0_deg}};

    int info_length = sizeof(info)/sizeof(info[0]);

    for(int i = 0; i < info_length; i++) {
        if (info[i].m_position == position) {
            return info[i];
        }
    }
    return info[0];
}

const Manipulator::PositionInfo& Manipulator::GetCustomPositionInfo() {
    // Get custom position data from shuffleboard, return starting position if uninitialised
    ManipulatorShuffleboard::PositionParams* custom_pos = m_shuffleboard->GetCustomPosition();
    if (custom_pos == nullptr) return GetPositionInfo(Position::Starting);

    // Update custom position from shuffleboard values
    m_custom_position_info.m_pivot_angle = custom_pos->pivot_angle;
    m_custom_position_info.m_wrist_angle = custom_pos->wrist_angle;
    m_custom_position_info.m_arm_extension = custom_pos->arm_extension;
    return m_custom_position_info;
}

Manipulator::Position Manipulator::GetCurrentPosition() {
    // Test whether we are at any of the valid positions and return them if so, otherwise
    // return Unknown.
    for (Position position = Position::FirstValid; position <= Position::LastValid; position = (Position)((int)position + 1)) {
        if (IsAtPosition(position, true)) {
            return position;
        }
    }
    return Position::Unknown;
}