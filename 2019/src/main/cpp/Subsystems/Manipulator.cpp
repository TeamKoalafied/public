//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "Mechanisms/Zucc.h"
#include "Mechanisms/Wrist.h"
#include "Mechanisms/Arm.h"
#include "Mechanisms/Pivot.h"
#include "Mechanisms/Intake.h"
#include "../RobotConfiguration.h"
#include "KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <math.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Manipulator::Manipulator() :
    TSingleton<Manipulator>(this),
    JoystickSubsystem("Manipulator", RC::kJoystickPortOperator) {

    m_zucc = new Zucc;
    m_wrist = new Wrist;
    m_arm = new Arm;
    m_pivot = new Pivot;
    m_intake = new Intake;

    m_wrist_offset_degrees = 0.0;
    m_pivot_auto_drive = false;
    m_auto_preset = NULL;
    m_intake_deployed = false; // Intake starts not deployed

    m_do_auto_drive = false;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    frc::Timer timer;

    m_zucc->Periodic(true);
    m_wrist->Periodic(true);
    m_arm->Periodic(true);
    m_pivot->Periodic(true);
    m_intake->Periodic(true);

    // Display timing for this function as smart dashboard updates can be a problem
    // taking a long time. This should be commented out except for testing.
    static int counter = 0;
    if (counter++ % 100 == 0) {
        std::cout << "Manipulator::Periodic() took " << (timer.Get() * 1000.0) << "ms\n";
    }

	frc::SmartDashboard::PutBoolean("AutoDrive", m_do_auto_drive);
    if (m_auto_preset != NULL) {
        frc::SmartDashboard::PutNumber("AutoLevel", m_auto_preset->m_level);        
    }

}

//==========================================================================
// Joystick Operation (from JoystickSubsystem)

void Manipulator::JoystickControlStarted() {
    JoystickSubsystem::JoystickControlStarted();
}

void Manipulator::DoJoystickControl() {
    frc::Joystick* joystick = GetJoystick();
    
    // Control one of the mechanisms, or the entire manipulator with the joystick. 
    // Only one of these should be called at a time! Different functions may use the
    // same axes or buttons for different thing and break teh robot!
    if (m_do_auto_drive) {
        DoAutomaticJoystickControl(joystick);
    } else {
        DoManualJoystickControl(joystick);
    }

    if (joystick->GetRawButtonPressed(RC::kJoystickLTrigButton)) {
        m_pivot_auto_drive = false;
        m_auto_preset = NULL;

        m_do_auto_drive = !m_do_auto_drive;
        std::cout << "CHANGING MODE m_do_auto_drive: " << m_do_auto_drive << "\n";
    }
//    m_pivot->TestDrivePivot(joystick);
//    m_wrist->TestDriveWrist(joystick);
//    m_arm->TestDriveArm(joystick);
//    m_zucc->TestDriveZucc(joystick);
//    m_intake->TestDriveIntake(joystick);
}

void Manipulator::JoystickControlStopped() {
    JoystickSubsystem::JoystickControlStopped();
}


//==============================================================================
// Setup and Shutdown

void Manipulator::Setup() {
    std::cout << "Manipulator::Setup()\n";

    // Setup all the mechanisms
    m_zucc->Setup();
    m_wrist->Setup();
    m_arm->Setup();
    m_pivot->Setup();
    m_intake->Setup();
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // Shutdown all the mechanisms
    m_zucc->Shutdown();
    m_wrist->Shutdown();
    m_arm->Shutdown();
    m_pivot->Shutdown();
    m_intake->Shutdown();
}

//==========================================================================
// Mechanism Access

void Manipulator::StartVacuum(){
    m_zucc->StartVacuum();
}

void Manipulator::HoldVacuum() {
    m_zucc->HoldVacuum();
}

bool Manipulator::GetVacuumObjectDetected() {
    return m_zucc->GetVacuumObjectDetected();
}

void  Manipulator::GotoVertical() {
        m_auto_preset = GetPresetForLevel(0);
        if (m_auto_preset != NULL) {
            m_pivot_auto_drive = true;
            m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
        }
}


//==========================================================================
// Joystick Control

void Manipulator::DoManualJoystickControl(frc::Joystick* joystick)
{
    // Drive the zucc using the POV control
	switch (joystick->GetPOV()) {
		case RC::kJoystickPovUp:    m_zucc->StartVacuum(); break;
		case RC::kJoystickPovDown:  m_zucc->StartExpel(); break;
		case RC::kJoystickPovLeft:  m_zucc->SetBallMode(); break;
		case RC::kJoystickPovRight: m_zucc->SetHatchMode(); break;
		default:
			m_zucc->HoldVacuum();
			break;
	}

    // Drive the wrist using the left Y axis
    double wrist_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
    if (fabs(wrist_drive) < RC::kJoystickDeadzone) wrist_drive = 0.0;
    if (wrist_drive != 0.0) {
	    m_wrist->ManualDriveWrist(wrist_drive);
		// std::cout << "Manual drive " << wrist_drive << "\n";
 	} else {
		if (!m_wrist->IsAngleSet()) {
			// Lock the wrist at the current position by doing close loop drive to it.
			double angle_degrees = m_wrist->GetWristAngleDegrees();
			m_wrist->DriveToAngleDegrees(angle_degrees, 1.0);
		}
	}

    bool arm_at_limit = EnforceMaxArmExtension();

    // Drive the arm using the triggers
    //  - left trigger extends
    //  - right trigger retracts
    double left_trigger = joystick->GetRawAxis(RC::kJoystickLeftTriggerAxis);
    double right_trigger = joystick->GetRawAxis(RC::kJoystickRightTriggerAxis);
    if (arm_at_limit) right_trigger = 0.0;
    if (left_trigger > 0.0 && right_trigger == 0.0) {
        m_arm->ManualDriveArm(-left_trigger);
    } else if (right_trigger > 0.0 && left_trigger == 0.0) {
        m_arm->ManualDriveArm(right_trigger);
    } else {
        // m_arm->ManualDriveArm(0.0);
        // Lock the arm at the current position by doing close loop drive to it.
        if (!m_arm->IsExtensionSet()) {
            int current_extension = m_arm->GetExtensionInch();
            m_arm->SetExtensionInch(current_extension, 1.0);
        }
    }

    // Drive the pivot using the right Y axis. Reverse the direction as that feels
    // more natural.
    double pivot_drive = -joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(pivot_drive) < RC::kJoystickDeadzone) {
        pivot_drive = 0.0;
    }
    pivot_drive = KoalafiedUtilities::PowerAdjust(pivot_drive, RobotConfiguration::kWristJoystickPower);

    // Apply limiting for the arm extension
    if (pivot_drive != 0.0) {
        pivot_drive = AdjustPivotVelocity(pivot_drive, false);
    }

    // Drive the pivot
    if (pivot_drive != 0.0) {
        // If manual drive just drive and record that it was manual
		m_pivot->ManualDrivePivot(pivot_drive);
        m_pivot_auto_drive = false;
 	} else {
        if (!m_pivot->IsAngleSet()) {
            // If we were doing manual drive then lock the pivot at the current
            // position by doing close loop drive to it.
            double angle_degrees = m_pivot->GetPivotAngleDegrees();
            m_pivot->DriveToAngleDegrees(angle_degrees, 1.0);
        }
	}

    // Drive the intake deploy using the B and X buttons. Only one intake position can be set.
    //  - B is retracted to the In or Vertical positions
    //  - X is Out position
    if (joystick->GetRawButton(RC::kJoystickXButton)) {
        // When retracting try to go to the In position, but if that is not allowed go to Vertical.
        if (!SetIntakePositionAvoidPivot(Intake::IntakePosition::In)) {
            SetIntakePositionAvoidPivot(Intake::IntakePosition::Vertical);
        }
    } else if (joystick->GetRawButton(RC::kJoystickBButton)) {
        SetIntakePositionAvoidPivot(Intake::IntakePosition::Out);
    }

    //  - A runs the roller as long as it is pressed
    //  - Y runs the roller in reverse as long as it is pressed
    if (joystick->GetRawButton(RC::kJoystickAButton)) {
        m_intake->StartIntakeRoller(false);
    } else if (joystick->GetRawButton(RC::kJoystickYButton)) {
        m_intake->StartIntakeRoller(true);
    } else {
        m_intake->StopIntakeRoller();
    }

    // Automatically adjust the wrist position to account for changes in the pivot angle
    DoAutoWristAdjust(wrist_drive != 0.0);

    // Update the soft limits on pivot position based on the position of the intake
    UpdatePivotSoftLimits();

    // Adjust zucc angle based on whether the intake is deployed
    //AutoAdjustZuccOffset();
}

double Lerp(double x1, double x2, double fraction) {
    return x1 * (1.0 - fraction) + x2 * fraction;
}

void Manipulator::DoAutoWristAdjust(bool wrist_driven_manually) {
    // Forward and reverse angle at which the wrist starts turning over the for the
    // other side
    const double kLevel3AngleForward = 15;
    const double kLevel3AngleBackward = -5;

    // "Boast" angle for stopping the wrist drooping when loaded with
    // a hatch.
    const double kBoastDroopAngle = 10;

    // Calculate the 'standard' wrist angle for the current pivot angle. This is the
    // angle that the wrist should be for the most common operations.
    double pivot_angle_degrees = m_pivot->GetPivotAngleDegrees();
    double standard_wrist_angle_degrees = 0.0;
    if (pivot_angle_degrees >= 0.0) {
        // Pivot is forward
        if (pivot_angle_degrees > kLevel3AngleForward) {
            // Pivot is reaching forward so the wrist should be perpendicular to the floor. It
            // needs to be 0 when the pivot is 90 and 90 when the pivot is 0.
            standard_wrist_angle_degrees = 90.0 - pivot_angle_degrees - kBoastDroopAngle;
        } else {
            // Pivot is transitioning as the arm crosses from one side to the other.
            double transition_wrist_angle_degrees = 90.0 - kLevel3AngleForward - kBoastDroopAngle;
            standard_wrist_angle_degrees = Lerp(0.0, transition_wrist_angle_degrees, pivot_angle_degrees / kLevel3AngleForward);
        }
    } else {
        // Pivot is backwards
        if (pivot_angle_degrees < kLevel3AngleBackward) {
            // Pivot is backwards so the wrist should be perpendicular to the floor. It
            // needs to be 0 when the pivot is -90 and -90 when the pivot is 0.
            standard_wrist_angle_degrees = -90.0 - pivot_angle_degrees + kBoastDroopAngle;
        } else {
            // Pivot is transitioning as the arm crosses from one side to the other.
            double transition_wrist_angle_degrees = -90.0 - kLevel3AngleBackward + kBoastDroopAngle;
            standard_wrist_angle_degrees = Lerp(0.0, transition_wrist_angle_degrees, pivot_angle_degrees / kLevel3AngleBackward);
        }
    }

    if (wrist_driven_manually) {
        // If the wrist is being driven manually then we do not adjust it. Instead
        // we update the offset between standard and where the user has placed it.
        double wrist_angle_degrees = m_wrist->GetWristAngleDegrees();
        m_wrist_offset_degrees = wrist_angle_degrees - standard_wrist_angle_degrees;
    } else {
        // If the wrist is not being manually driven we automatically update its angle,
        // but maintaining any offset that the user has applied by driving it manually.
        double wrist_angle_degrees = standard_wrist_angle_degrees + m_wrist_offset_degrees;
        m_wrist->DriveToAngleDegrees(wrist_angle_degrees, 1.0);
    }
}

void Manipulator::UpdatePivotSoftLimits() {
    switch (m_intake->GetIntakePosition()) {
        case Intake::IntakePosition::In:
            // Pivot can move over its forward range if the intake is in
            m_pivot->SetSoftLimitsAngleDegrees(RC::kPivotMaximumForwardDegrees, -5.0);

            // If the pivot is hitting the soft limit for the In position then move the intake to Vertical.
            // When the intake gets to Vertical the soft limit will change allowing the pivot to move.
            if (m_pivot->GetReverseSoftLimitHit()) {
                SetIntakePositionAvoidPivot(Intake::IntakePosition::Vertical);
            }
            break;
        case Intake::IntakePosition::Out:
            // Pivot can move over its full range if the intake is Out
            m_pivot->SetSoftLimitsAngleDegrees(RC::kPivotMaximumForwardDegrees, RC::kPivotMaximumReverseDegrees);
            break;
        case Intake::IntakePosition::Vertical:
            // Pivot can move over its backward range if the intake is Vertical
            m_pivot->SetSoftLimitsAngleDegrees(5.0, RC::kPivotMaximumReverseDegrees);

            // If the pivot is hitting the soft limit for the Vertical position then move the intake to In.
            // When the intake gets to In the soft limit will change allowing the pivot to move.
            if (m_pivot->GetForwardSoftLimitHit()) {
                SetIntakePositionAvoidPivot(Intake::IntakePosition::In);
            }
            break;
        case Intake::IntakePosition::Moving:
            // Leave the limits as they are and hope for the best
            break;
    }
}

bool Manipulator::EnforceMaxArmExtension() {
    // Calculate the 'maximum' arm extension for the current pivot angle. This is the
    // maximum extension the arm can have a still stay inside a desired distance from
    // the frame perimeter.
    // The rules require staying within 30" of the perimeter, be we are aiming for 15".
    double pivot_angle_degrees = m_pivot->GetPivotAngleDegrees();

    // If we are near to vertical then there is no limit that needs to be applied. This
    // prevents divide by zero errors below.
    if (fabs(pivot_angle_degrees) < 5.0) return false;

    // Calculate the value. Diagram in KoalafiedShare > Software > 2019 > Inter-Mechanism Constraints
    double maximum_arm_extension_inch = 0.0;
    if (pivot_angle_degrees >= 0.0) {
        maximum_arm_extension_inch = (RC::kForwardMaximumExtensionInch + RC::kArmLengthInch)/cos((90.0 - pivot_angle_degrees)*M_PI/180.0) - RC::kArmLengthInch;
    } else {
        maximum_arm_extension_inch = (RC::kBackwardMaximumExtensionInch + RC::kArmLengthInch)/cos((90.0 + pivot_angle_degrees)*M_PI/180.0) - RC::kArmLengthInch;
        if (maximum_arm_extension_inch < 0.0) maximum_arm_extension_inch = 0.0;
    }

    frc::SmartDashboard::PutNumber("ArmMaxExtension", maximum_arm_extension_inch);

    // If the extension is beyond the maximum, drive back toward the it
    double arm_extension_inch = m_arm->GetExtensionInch();
    if (arm_extension_inch > maximum_arm_extension_inch) {
        // std::cout << "maximum_arm_extension_inch " << maximum_arm_extension_inch << "\n";
        m_arm->SetExtensionInch(maximum_arm_extension_inch, 1.0);
        return true;
    }
    return false;
}

 double Manipulator::AdjustPivotVelocity(double pivot_drive, bool auto_drive) {
    // Original constraint looked like this:
    // // Constraint #3 - slow the pivot down the more the arm extends
    // double adjusted_velocity = ((RC::kArmMaximumExtensionInch - m_arm->GetExtensionInch())/RC::kArmMaximumExtensionInch) * 0.5 + 0.5;
    // m_pivot->ManualDrivePivot(pivot_drive * adjusted_velocity);

    // Phil's code snippet
    // double adjusted_velocity;
    // if (pivot_drive != 0.0) {
    //     // If the arm is high allow it to move at a reduced speed if the angle is 'safe'.
    //     // If the angle is 'unsafe' lock the pivot until the driver retracts the arm.
    //         if ((m_arm ->GetExtensionInch() > RC:kArmHighExtensionThreshold) &&
    //         ((m_pivot->GetPivotAngleDegrees() > RC:kPivotForwardHighExtensionAngle) ||
    //             (m_pivot->GetPivotAngleDegrees() < RC:kPivotReverseHighExtensionAngle))) {
    //                 adjusted_velocity = 0;
    //         } else {
    //             adjusted_velocity = ((RC::kArmMaximumExtensionInch - m_arm->GetExtensionInch ())/RC::kArmMaximumExtensionInch) * 0.5 + 0.5;
            
    //         }
    //         m_pivot->ManualDrivePivot(pivot_drive * adjusted_velocity); // std::cout << "Manual drive " << pivot_drive << " adjust " << adjusted_velocity << "\n"; } else { if (!m_pivot->IsAngleSet()) { //ManualDrivePivot(0.0); //std::cout << "Zeroing drive " << "\n"; // Lock the pivot at the current position by doing close loop drive to it. double angle_degrees = m_pivot->GetPivotAngleDegrees(); m_pivot->DriveToAngleDegrees(angle_degrees, 1.0); } 
    // }


    double arm_extension_inch = m_arm->GetExtensionInch();
    double pivot_angle_degrees = m_pivot->GetPivotAngleDegrees();

    double const kSafeArmExtensionInch = 2.0;
    double const kPivotSpeedAtFullExtension = 0.3;

    // Full extension limit angles. Between these pivot angles full extension is legal.
    double const kPivotForwardHighExtensionAngle = 30.0;
    double const kPivotReverseHighExtensionAngle = -18.0;

    double const kArmHighExtensionThreshold = 15.0;

    // Calculate an adjusted drive so that the pivot slows down as the arm extends
    double velocity_adjust = Lerp(1.0, kPivotSpeedAtFullExtension, arm_extension_inch/RC::kArmMaximumExtensionInch);
    pivot_drive *= velocity_adjust;

    // If the arm extension is very small just allow full movement
    if (arm_extension_inch < kSafeArmExtensionInch) {
        return pivot_drive;
    }

    // If the arm is moving towards vertical then allow the movement
    if ((pivot_drive > 0.0 && pivot_angle_degrees < 0.0) || (pivot_drive < 0.0 && pivot_angle_degrees > 0.0)) {
        return pivot_drive;
    }

    // If the movement is not one of the two safe cases above, and we are doing auto drive
    // then lock the pivot.
    if (auto_drive) {
        return 0.0;
    }

    // For the current arm extension calculate forward and reverse pivot angles at
    // which the arm will be at the maximum over hang. Note that in the forward
    // case it is possible for the arm extension to be OK for any angle (in which
    // case the ratio is greater than 1.0).
    double forward_ratio = (RC::kForwardMaximumExtensionInch + RC::kArmLengthInch)/
                                      (arm_extension_inch + RC::kArmLengthInch);
    double reverse_ratio = (RC::kBackwardMaximumExtensionInch + RC::kArmLengthInch)/
                                      (arm_extension_inch + RC::kArmLengthInch);
    if (forward_ratio > 1.0) forward_ratio = 1.0;
    if (reverse_ratio > 1.0) reverse_ratio = 1.0;

    double forward_limit_angle = 90.0 - (180.0/M_PI)*acos(forward_ratio);
    double reverse_limit_angle = -90.0 + (180.0/M_PI)*acos(reverse_ratio);

    // If the pivotis outside the safe limits stop it moving, otherwise allow it it
    if (pivot_angle_degrees > forward_limit_angle || pivot_angle_degrees < reverse_limit_angle) {
        return 0.0;
    } else {
        return pivot_drive;
    }

    // If the arm is over the high extension threshold, and it is outside the safe limits stop
    //  it moving, otherwise allow it it
//    if (arm_extension_inch > kArmHighExtensionThreshold &&
//        (pivot_angle_degrees > kPivotForwardHighExtensionAngle || pivot_angle_degrees < kPivotReverseHighExtensionAngle)) {
//        return 0.0;
//    } else {
//        return pivot_drive;
//    }
}

bool Manipulator::SetIntakePositionAvoidPivot(Intake::IntakePosition position) {
    // If the intake is switching to a different position then test if it is allowed
    Intake::IntakePosition current_position = m_intake->GetIntakePosition();
    if (position != current_position) {
        double pivot_angle_degrees = m_pivot->GetPivotAngleDegrees();

        // There are 3 possible movements, because the direction does not matter, just the end points.
        // Just enumerate them as that is easy to understand (if a little tedious).
        bool allowed = false;
        if ((position == Intake::IntakePosition::In       && current_position == Intake::IntakePosition::Vertical) ||
            (position == Intake::IntakePosition::Vertical && current_position == Intake::IntakePosition::In)) {
            // Going between In and Vertical. Pivot must be fairly vertical.
            allowed = pivot_angle_degrees < 25.0 && pivot_angle_degrees > -10.0;
        } else if ((position == Intake::IntakePosition::Vertical && current_position == Intake::IntakePosition::Out) ||
                   (position == Intake::IntakePosition::Out      && current_position == Intake::IntakePosition::Vertical)) {
            // Going between Vertical and Out. Pivot must not be low and forward.
            allowed = pivot_angle_degrees < 25.0;
        } else if ((position == Intake::IntakePosition::In  && current_position == Intake::IntakePosition::Out) ||
                   (position == Intake::IntakePosition::Out && current_position == Intake::IntakePosition::In)) {
            // Going between In and Out. Pivot must be within both the ranges above. This is actually
            // just the same as the In and Vertical case.
            allowed = pivot_angle_degrees < 30.0 && pivot_angle_degrees > -10.0;
        }

        if (!allowed) {
            std::cout << "Intake cannot move due to pivot position";
            return false;
        }
    }

    // Moving the intake is OK so do it
    m_intake->SetIntakePosition(position);
    return true;
}

void Manipulator::DoAutomaticJoystickControl(frc::Joystick* joystick) {
    // Drive the zucc using the POV control
	switch (joystick->GetPOV()) {
		case RC::kJoystickPovUp:    m_zucc->StartVacuum(); break;
		case RC::kJoystickPovDown:  m_zucc->StartExpel(); break;
		case RC::kJoystickPovLeft:  m_zucc->SetBallMode(); break;
		case RC::kJoystickPovRight: m_zucc->SetHatchMode(); break;
		default:
			m_zucc->HoldVacuum();
			break;
	}

    // Drive the wrist using the left Y axis
    double wrist_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
    if (fabs(wrist_drive) < RC::kJoystickDeadzone) wrist_drive = 0.0;
    if (wrist_drive != 0.0) {
	    m_wrist->ManualDriveWrist(wrist_drive);
		// std::cout << "Manual drive " << wrist_drive << "\n";
 	} else {
		if (!m_wrist->IsAngleSet()) {
			// Lock the wrist at the current position by doing close loop drive to it.
			double angle_degrees = m_wrist->GetWristAngleDegrees();
			m_wrist->DriveToAngleDegrees(angle_degrees, 1.0);
		}
	}

    bool arm_at_limit = EnforceMaxArmExtension();

    // Drive the arm using the triggers
    //  - left trigger extends
    //  - right trigger retracts
    double left_trigger = joystick->GetRawAxis(RC::kJoystickLeftTriggerAxis);
    double right_trigger = joystick->GetRawAxis(RC::kJoystickRightTriggerAxis);
    if (arm_at_limit) right_trigger = 0.0;
    if (left_trigger > 0.0) {
        m_arm->SetExtensionInch(0.0, 1.0);
    }
    if (right_trigger > 0.0 && m_auto_preset != NULL) {
        m_arm->SetExtensionInch(m_auto_preset->m_extension_inch, 1.0);
    }

    // Drive the pivot using the right Y axis. Reverse the direction as that feels
    // more natural. The X axis is used for auto drive, which tries to stop at set
    // angles. If the joystick is moved diagonally the larger movement is used.
    double pivot_drive_auto = -joystick->GetRawAxis(RC::kJoystickRightYAxis);
//    if (fabs(pivot_drive_auto) < RC::kJoystickDeadzone) {
    if (fabs(pivot_drive_auto) < 0.1) { // Operator joystick seems to have a big deadzone
        pivot_drive_auto = 0.0;
    }
    pivot_drive_auto = KoalafiedUtilities::PowerAdjust(pivot_drive_auto, RobotConfiguration::kWristJoystickPower);

    // Apply limiting for the arm extension
    if (pivot_drive_auto != 0.0) {
        pivot_drive_auto = AdjustPivotVelocity(pivot_drive_auto, true);

        // If the auto drive is ever stopped by the arm extension then we clear the
        // auto drive flag so that we don't move to some snap position.
        if (pivot_drive_auto == 0.0) m_pivot_auto_drive = false;
    }

    // Drive the pivot
    if (pivot_drive_auto != 0.0) {
        // If auto drive just drive as for manual, but calculate and record the
        // angle that we should auto drive to when the operator stops pushing the stick
		m_pivot->ManualDrivePivot(pivot_drive_auto);
        m_auto_preset = CalculatePivotAutoPreset(pivot_drive_auto);
        frc::SmartDashboard::PutNumber("PivotAutoAngle", m_auto_preset->m_angle_degrees);
        frc::SmartDashboard::PutNumber("PivotAutoExtension", m_auto_preset->m_extension_inch);
        m_pivot_auto_drive = true;
 	} else {
        if (!m_pivot->IsAngleSet()) {
            if (m_pivot_auto_drive && m_auto_preset != NULL) {
                // If we were doing auto drive then drive to the required set angle
                m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
            } else {
                // If we were doing manual drive then lock the pivot at the current
                // position by doing close loop drive to it.
                double angle_degrees = m_pivot->GetPivotAngleDegrees();
                m_pivot->DriveToAngleDegrees(angle_degrees, 1.0);
            }
        }
	}

    int side = (m_pivot->GetPivotAngleDegrees() < 0.0) ? -1 : +1;
    if (joystick->GetRawButton(RC::kJoystickBButton)) {
        m_auto_preset = GetPresetForLevel(3 * side);
        if (m_auto_preset != NULL) {
            m_pivot_auto_drive = true;
            m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
        }
    } else if (joystick->GetRawButton(RC::kJoystickYButton)) {
        m_auto_preset = GetPresetForLevel(2 * side);
        if (m_auto_preset != NULL) {
            m_pivot_auto_drive = true;
            m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
        }
    } else if (joystick->GetRawButton(RC::kJoystickXButton)) {
        m_auto_preset = GetPresetForLevel(1 * side);
        if (m_auto_preset != NULL) {
            m_pivot_auto_drive = true;
            m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
        }
    } else if (joystick->GetRawButton(RC::kJoystickAButton)) {
        m_auto_preset = GetPresetForLevel(0);
        if (m_auto_preset != NULL) {
            m_pivot_auto_drive = true;
            m_pivot->DriveToAngleDegrees(m_auto_preset->m_angle_degrees, 1.0);
        }
    }

/*
    // Drive the intake deploy using the B and X buttons. Only one intake position can be set.
    //  - B is retracted to the In or Vertical positions
    //  - X is Out position
    if (joystick->GetRawButton(RC::kJoystickXButton)) {
        // When retracting try to go to the In position, but if that is not allowed go to Vertical.
        if (!SetIntakePositionAvoidPivot(Intake::IntakePosition::In)) {
            SetIntakePositionAvoidPivot(Intake::IntakePosition::Vertical);
        }
    } else if (joystick->GetRawButton(RC::kJoystickBButton)) {
        SetIntakePositionAvoidPivot(Intake::IntakePosition::Out);
    }

    //  - A runs the roller as long as it is pressed
    //  - Y runs the roller in reverse as long as it is pressed
    if (joystick->GetRawButton(RC::kJoystickAButton)) {
        m_intake->StartIntakeRoller(false);
    } else if (joystick->GetRawButton(RC::kJoystickYButton)) {
        m_intake->StartIntakeRoller(true);
    } else {
        m_intake->StopIntakeRoller();
    }
*/
    // Automatically adjust the wrist position to account for changes in the pivot angle
    DoAutoWristAdjust(wrist_drive != 0.0);

    // Update the soft limits on pivot position based on the position of the intake
    UpdatePivotSoftLimits();

    // Adjust zucc angle based on whether the intake is deployed
    //AutoAdjustZuccOffset();
}

const Manipulator::Preset* Manipulator::CalculatePivotAutoPreset(double pivot_drive_auto) {

    // Determine which group of presets to us based on the object being
    // manipulated.
    int total_presets = 0;
    const Preset* presets = GetPresets(total_presets);

    // Find the preset that applied for the current pivot angle
    double pivot_angle = m_pivot->GetPivotAngleDegrees();
    int preset_index = 0;
    for (preset_index = 0; preset_index < total_presets; preset_index++) {
        if (pivot_angle < presets[preset_index].m_angle_degrees) break;
    }

    // If we are before the first angle then it is the only choice
    if (preset_index == 0) return &presets[0];

    // If we are after the last angle then it is the only choice
    if (preset_index == total_presets) return &presets[total_presets - 1];

    // Calculate the proportional position that we are between the two angles
    double angle_below = presets[preset_index - 1].m_angle_degrees;
    double angle_above = presets[preset_index].m_angle_degrees;
    const Preset* preset_below = &presets[preset_index - 1];
    const Preset* preset_above = &presets[preset_index];
    double position = (pivot_angle - angle_below)/(angle_above - angle_below);

    // If we are very close to one angle then use it. This means that if we drive slightly
    // past a set angle we do not continue to the next one.
    double kAngleGrabFraction = 0.1;
    if (position < kAngleGrabFraction) return preset_below;
    if (position > 1.0 - kAngleGrabFraction) return preset_above;

    // Finally pick the angle in the direction we are moving
    if (pivot_drive_auto > 0.0) return preset_above;
    else                        return preset_below;
}

const Manipulator::Preset* Manipulator::GetPresets(int& total_presets) {
    // Presets for picking and placing hatches
    const static Preset HATCH_PRESETS[] = {
        { -81.2,  0.0, -1 }, // Back level 1
        { -12.6,  7.8, -2 }, // Back level 2
        {  -7.0, 32.8, -3 }, // Back level 3
        {   0.0,  0.0,  0 }, // Vertical
        {  20.4, 38.8,  3 }, // Forward level 3
        {  30.2,  9.8,  2 }, // Forward level 2
        {  83.0,  0.0,  1 }  // Forward level 1 - 85.8, 85.7
    };

    // Presets for placing balls
    const static Preset BALL_PRESETS[] = {
        { -90.0,  0.0, -1 }, // Back level 1
        { -50.0,  5.0, -2 }, // Back level 2
        { -10.0, 10.0, -3 }, // Back level 3
        {   0.0,  0.0,  0 }, // Vertical
        {  30.0, 10.0,  3 }, // Forward level 3
        {  60.0,  5.0,  2 }, // Forward level 2
        {  83.0,  0.0,  1 }  // Forward level 1
    };

    // Presets for picking up balls. The grab ball one is critical, the
    // others are not so important (just required to lift the pivot
    // so the intake can be retracted and to avoid widely spaced presets).
    const static Preset BALL_INTAKE_PRESETS[] = {
        { -90.0,  0.0, -1 }, // Back level 1
        { -50.0,  5.0, -2 }, // Back level 2
        { -10.0, 10.0, -3 }, // Back level 3
        {   0.0,  0.0,  0 }, // Vertical
        {  30.0, 10.0,  3 }, // Forward level 3
        {  60.0,  5.0,  2 }, // Forward level 2
        {  75.0,  0.0,  1 }  // Forward grab ball
    };

    // Determine which group of presets to us based on the object being
    // manipulated.
    if (m_zucc->GetMode() == Zucc::Mode::Hatch) {
        total_presets = sizeof(HATCH_PRESETS)/sizeof(HATCH_PRESETS[0]);
        return HATCH_PRESETS;
    } else {
        // The intake is out there are presets for picking the ball
        if (m_intake->GetIntakePosition() == Intake::IntakePosition::Out) {
            total_presets = sizeof(BALL_INTAKE_PRESETS)/sizeof(BALL_INTAKE_PRESETS[0]);
            return BALL_INTAKE_PRESETS;
        } else {
            total_presets = sizeof(BALL_PRESETS)/sizeof(BALL_PRESETS[0]);
            return BALL_PRESETS;
        }
    }
}

const Manipulator::Preset* Manipulator::GetPresetForLevel(int level) {
    // Determine which group of presets to us based on the object being
    // manipulated.
    int total_presets = 0;
    const Preset* presets = GetPresets(total_presets);

    for (int i = 0; i < total_presets; i++) {
        if (presets[i].m_level == level) {
            return &presets[i];
        }
    }
    return NULL;
}

void Manipulator::AutoAdjustZuccOffset() {
    bool intake_deployed = m_intake->GetIntakePosition() == Intake::IntakePosition::Out;
    if (intake_deployed && !m_intake_deployed) {
        // Intake has just been deployed. Rotate the wrist forward to pick the ball.
        m_wrist_offset_degrees = 90;
    } else if (!intake_deployed && m_intake_deployed) {
        // Intake has just been retracted. Rotate the wrist backward to place the ball.
        m_wrist_offset_degrees = 90;
    }
    m_intake_deployed = intake_deployed;
}
