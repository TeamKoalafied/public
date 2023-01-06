//
// FindTargetControl.cpp
//

#include "FindTargetControl.h"

#define _USE_MATH_DEFINES
#include "../HapticController.h"
#include "../RobotConfiguration.h"
#include "../Subsystems/DriveBase.h"
#include "../Subsystems/Manipulator.h"


#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <math.h>

namespace RC = RobotConfiguration;

//==============================================================================
// Construction and Destruction

FindTargetControl::FindTargetControl(DriveBase& drive_base, Manipulator& manipulator) :
    m_drive_base(drive_base),
    m_manipulator(manipulator) {

    //m_state = State::kIdle;
    m_target_history_index = 0;
    for (int i = 0; i < HISTORY_LENGTH; i++) {
        m_target_headings[i] = kErrorHeading;
        m_target_distances_m[i] = kErrorDistance;
    }
}

FindTargetControl::~FindTargetControl() {
}


//==============================================================================
// Operation

void FindTargetControl::UpdateTargetHeading() {
	std::shared_ptr< nt::NetworkTable > table = nt::NetworkTableInstance::GetDefault().GetTable("pivision");
	float tx = table->GetNumber("tx", 0.0);  // degrees (-27 to 27 for limelight1)
	float ty = table->GetNumber("ty", 0.0);  // 
	bool target_found = (0.0f != table->GetNumber("tv", 0.0));  // 0.0 unless the target is detected

    // Calculate the current heading of the target
    double target_heading = kErrorHeading;
    if (target_found) {
        target_heading = - tx;
    }

    // Calculate the current distance of the target
    double target_distance_m = target_found ? ty : kErrorDistance;

    // Record the current heading and distance in the circular buffers
    m_target_headings[m_target_history_index] = target_heading;
    m_target_distances_m[m_target_history_index] = target_distance_m;
    m_target_history_index++;
    if (m_target_history_index == HISTORY_LENGTH) m_target_history_index = 0;

    // Get the sum and count of all the valid heading in the buffer
    double total_target_heading = 0.0;
    double total_target_distance = 0.0;
    int valid_count = 0;
    for (int i = 0; i < HISTORY_LENGTH; i++) {
        double heading = m_target_headings[i];
        double distance = m_target_distances_m[i];
        if (heading != kErrorHeading) {
            total_target_heading += heading;
            total_target_distance += distance;
            valid_count++;
        }
    }

    // If at least half the headings are valid then regard the heading as valid
    if (valid_count >= HISTORY_COUNT_FOR_VALID) {
        m_target_valid = true;
        m_vision_target_heading = total_target_heading / valid_count;
        m_vision_target_distance_m = total_target_distance / valid_count;
    }
    else {
        m_target_valid = false;
    }

    frc::SmartDashboard::PutNumber("FindTargetValidCount", valid_count);
    frc::SmartDashboard::PutNumber("FindTargetHeading", m_vision_target_heading);
    frc::SmartDashboard::PutNumber("FindTargetDistanceM", m_vision_target_distance_m);
    frc::SmartDashboard::PutNumber("FindTargetValid", m_target_valid);

    // Calculate the heading and distance to the target using dead reckoning (i.e. the position
    // that the drivebase thinks it is in)
    const double TARGET_X_M = -2.0;
    const double TARGET_Y_M = 0.0;
    double position_x_m;
    double position_y_m;
    double heading_degrees;
    m_drive_base.GetPositionM(position_x_m, position_y_m, heading_degrees);
    double dx = position_x_m - TARGET_X_M;
    double dy = position_y_m - TARGET_Y_M;
    m_tracking_target_distance_m = ::sqrt(dx*dx + dy*dy);
    int target_heading_absolute = ::atan2(dy, dx) * 180.0 / M_PI;

    // Calculate the target relative to the robot.
    // (I think this actually calculates relative to out the back of the robot, which is 0 for the turret)
    m_tracking_target_heading = NormaliseAngleDiffDegrees(target_heading_absolute, heading_degrees);

    frc::SmartDashboard::PutNumber("Tracking Target Distance (ft)", m_tracking_target_distance_m/(12*0.0254));
    frc::SmartDashboard::PutNumber("Tracking Target Heading", m_tracking_target_heading);
}

FindTargetControl::FindResult FindTargetControl::RotateToTargetIdle(double& distance_ft) {
    return RotateToTarget(30, 0.4, distance_ft);
}

FindTargetControl::FindResult FindTargetControl::RotateToTargetShooting(double& distance_ft) {
    return RotateToTarget(0, 0.8, distance_ft);
}

FindTargetControl::FindResult FindTargetControl::RotateToTarget(int reverse_tolerance_degrees, double max_turret_speed, double& distance_ft) {

    double turret_angle_degrees = m_manipulator.GetTurretAngleDegrees();

    // If vision is valid use that to determine the target position, otherwise use drivebase tracking
    double target_turret_angle;
    if (m_target_valid) {
        // The vision target angle is relative to the current turret position
        target_turret_angle = turret_angle_degrees + m_vision_target_heading;
        distance_ft = m_vision_target_distance_m/(12*0.0254);
    } else {
        // Tracking with drive base dead reckoning is an absolute turret angle
        target_turret_angle = NormaliseAngleDegrees(m_tracking_target_heading);
        distance_ft = m_tracking_target_distance_m/(12*0.0254);
    }

    frc::SmartDashboard::PutNumber("Tracking Turret Heading", target_turret_angle);

    // Do blind spot detection and blind spot rejection
    // TODO Have not worked this out. The driver and operator will just need to be aware of it
    if (target_turret_angle) {
    }

    // Drive the turret to the desired angle
    if (m_manipulator.SetTurretAngleDegrees(target_turret_angle, max_turret_speed)) {
        // Return on target, either with vision or without
        return m_target_valid ? FindResult::kOnTargetVision : FindResult::kOnTargetNoVision;
    } else {
        // Return rotating to the target, either with vision or without
        return m_target_valid ? FindResult::kRotatingToTargetVision : FindResult::kRotatingToTargetVision;
    }
}
/*
bool FindTargetControl::DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller,
                                             HapticController* haptic_controller2) {
    // Update the information about the target heading
//    bool old_target_valid = m_target_valid;
//    UpdateTargetHeading();

    // If the driver is pressing the Y button attempt to rotate to the target
    if (joystick->GetRawButton(RC::kJoystickYButton)) {
        if (m_target_valid) {
            if (m_state != State::kReachedTarget) {
                // The target is valid and we have not reached it yet so perform the necessary
                // rotation.
                if (RotateToTarget()) {
                    // The drivebase is now pointing at the target so do a long buzz of haptic
                    // feedback to notify the driver and operator.
                    haptic_controller->DoContinuousFeedback(1.0, 1.0);
                    haptic_controller2->DoContinuousFeedback(1.0, 1.0);

                    // Set the state to record that we reached the target. We won't rotate
                    // anymore unless the driver releases the button and pushes it again.
                    m_state = State::kReachedTarget;
                }
                else {
                    // Record that we ar rotating to the target and return false so that
                    // normal driver control is NOT performed
                    m_state = State::kRotatingToTarget;
                    return false;                  
                }
            }
        }
        else {
            // The target is not valid, so do a double buzz of haptic feedback to notify the driver and operator
            if (m_state != State::kSignalNoTarget) {
                static double PATTERN[10] = { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
                haptic_controller->DoFeedback(PATTERN, 10);
                haptic_controller2->DoFeedback(PATTERN, 10);

                m_state = State::kSignalNoTarget;
            }
        }
    }
    else {
        // Driver is not is not pressing the button so reset to the idle state
        m_state = State::kIdle;

        // If the target has just become valid do a short buzz of haptic feedback to indicate
        // that we have a lock on the target.
        // if (m_target_valid && !old_target_valid) {
        //     //haptic_controller->DoContinuousFeedback(0.5, 1.0);
        // }
    }

    // We are not rotating the drivebase to the target so return true so that
    // normal driver control is performed
    return true;
}*/

// bool FindTargetControl::AutoRotateToTarget() {
//    // UpdateTargetHeading();
//     if (m_target_valid) {
//         if (RotateToTarget()) {
//             // The drivebase is now pointing at the target so do a long buzz of haptic
//             // feedback to notify the driver.
//             return true;
//         } else {
//             return false;
//         }
//     }
//     return true; // For testing without the target
// //    return false;
// }

bool FindTargetControl::GetTargetDistance(double& distance) const {
    distance = m_vision_target_distance_m;
    return  m_target_valid;
}


//==============================================================================
// Dashboard Setup

void FindTargetControl::SetupDashboard() {
    // Values for VisionFindTarget pure vision feedback (open loop motor control)                                                         
    // TODO I don't think these are used anymore
    // frc::SmartDashboard::PutNumber("VisionTrackX", 3.0);
    // frc::SmartDashboard::PutNumber("VisionTrackY", 0.0);
    // frc::SmartDashboard::PutNumber("VisionTrackHeading", 0.0);
    frc::SmartDashboard::PutNumber("VisionKp", 0.006);         // Start small and double until overshoot                                          
    frc::SmartDashboard::PutNumber("VisionMinRotation", 0.03); // Experiments results 0.25 for wood, 0.33 carpet                                  
    frc::SmartDashboard::PutNumber("VisionMaxRotation", 0.3);  // Use to limit max speed when error is large                                      
}


//==============================================================================
// Implementation
/*
bool FindTargetControl::RotateToTarget() {
	// See Robot.cpp for initial settings. The defaults here are 0.0 if no connection
	float kp = frc::SmartDashboard::GetNumber("VisionKp", 0.0);						
	float minRotation = frc::SmartDashboard::GetNumber("VisionMinRotation", 0.0);	
	float maxRotation = frc::SmartDashboard::GetNumber("VisionMaxRotation", 0.0);	

	double current_heading = m_drive_base.GetPigeonHeading();


    double offset = m_target_heading - current_heading;
  
    while (offset < -180.0) offset += 360.0;
    while (offset > 180) offset -= 360.0;

    // Start the rotate, and give it 100ms to settle
    if ((offset < -0.5) || (offset > 0.5)) {
        // Give motors a little power even if error is small
        double rotation;
        if (offset > 0.0) {
            rotation = kp*offset + minRotation;
        } else {
            rotation = kp*offset - minRotation;
        }

        // Clip the maximum rotation for safety!
        if (rotation > maxRotation) rotation = maxRotation;
        if (rotation < -maxRotation) rotation = -maxRotation;

        //std::cout << "Vision: offset " << offset << "rotation " << rotation << std::endl; //debug
        m_drive_base.ArcadeDriveForVision(0.0, rotation);

        return false;
    } else {
        return true;
    }
}*/

double FindTargetControl::NormaliseAngleDegrees(int angle_degrees) {
	while (angle_degrees > 180.0)  angle_degrees -= 360.0;
	while (angle_degrees < -180.0) angle_degrees += 360.0;
    return angle_degrees;
}

double FindTargetControl::NormaliseAngleDiffDegrees(int final_angle_degrees, int initial_angle_degrees) {
        final_angle_degrees = NormaliseAngleDegrees(final_angle_degrees);
        initial_angle_degrees = NormaliseAngleDegrees(initial_angle_degrees);
        return NormaliseAngleDegrees(final_angle_degrees - initial_angle_degrees);
}
