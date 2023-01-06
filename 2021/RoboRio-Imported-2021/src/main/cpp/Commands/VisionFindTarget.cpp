//==============================================================================
// VisionFindTarget.cpp
//==============================================================================
//
// Overview
// Reflective tape can be found by shining a strong coloured light on it, and
// capturing underexposed video from a position behind the light. The brightness
// can be thresholded and the colour used to filter out other reflections and
// lights.
// The detected object can be further filtered based on size, orientation, lines
// positions of lines etc. The capture + filter process may take up to 40ms, but
// Limelight can do this faster. Our main loop runs at 20ms, so the data is
// delayed regardless.
// The field of view, resolution and the offset of the detected object can be
// used to compute the angle that the target is offset from the robot's heading.
// The offset is the angle the robot needs to turn to face the target. A
// positive offset means the robot should turn right to bring the target back to
// centre. A negative offset requires a left turn. The offset is scaled by Kp,
// added to a minimum speed (the highest value that does not cause the robot to
// move), and clipped to a maximum speed.
// Important considerations:
//  * RobotConfiguration::kDriveMotorNominalOutput is used to set the minimum
//    speed of the drive motors during DriveBase initialisation. The same value
//    is used for forward and backward on left and right motors. But this does
//    NOT appear to have any effect. minRotation is much more effective.
//  * Different floor coverings have different minimal speeds.
//  * High rotational speeds are too fast for the vision feedback to respond in
//    time. The delayed response gives larger errors requiring faster speeds to
//    correct (proportional control). To fix this, clip the maximum rotational
//    speed.
//
// Recommended procedure.
//  1 On new floor covering run the calibration script and note the speeds where
//    the robot just starts to move
//  2 Use these to set the nominal speeds for the drive base motors
//  3 Adjust Kp to give the best response for angles < 10 degrees
//  4 Try larger errors and reduce maximum rotation speed until response is stable
//
#include "VisionFindTarget.h"
#include "../RobotConfiguration.h"

#include "../Subsystems/DriveBase.h"
#include <frc/Joystick.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

//==============================================================================
// Construction

VisionFindTarget::VisionFindTarget() :
    frc::Command("VisionFindTarget", 5.0) {

    // This command requires the drive base. Vision is done via network tables
    Requires(&DriveBase::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void VisionFindTarget::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "VisionFindTarget::Initialize()\n";

    m_mode = kvision;
	m_next_mode = kdone;
	m_timer = 0;
	m_target_bearing = DriveBase::GetInstance().GetPigeonHeading();
}

void VisionFindTarget::Execute() {
    // Called repeatedly when this Command is scheduled to run

	// Derived from docs.limelightvision.io/en/latest/cs_aiming.html
	double rotation = 0.0f;
	DriveBase& drive_base = DriveBase::GetInstance();
	
	// See Robot.cpp for initial settings. The defaults here are 0.0 if no connection
	float kp = frc::SmartDashboard::GetNumber("VisionKp", 0.0);						// 0.017 or higher
	float minRotation = frc::SmartDashboard::GetNumber("VisionMinRotation", 0.0);	// 0.33 on carpet, 0.25 on wood
	float maxRotation = frc::SmartDashboard::GetNumber("VisionMaxRotation", 0.0);	// 0.7

	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("pivision");
//	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	float tx = table->GetNumber("tx", 0.0);  // degrees (-27 to 27 for limelight1)
	bool object_found = (0.0f != table->GetNumber("tv", 0.0));  // 0.0 unless the target is detected
	double heading = -drive_base.GetPigeonHeading();  // positive tx is clockwise, so flip sign here
	float offset = 0;

    switch (m_mode) {
	case ktimer:
		// Waits multiples of main loop time (20ms). Assumes m_timer >= 1 at start.
		if (m_timer > 0)
			m_timer--;
		if (m_timer <= 0)
			m_mode = m_next_mode;
	break;

	// This approach uses tx as the offset in a standard PID controller loop with I=0, D=0
	// If the object is not found, we estimate the offset based on how far we have turned
	case kvision:
		if (object_found) {
			// compute the offset to aim for
			offset = tx;
			// Compute the target bearing for later
			m_target_bearing = heading;
			if (heading != DriveBase::kHeadingError)
				m_target_bearing += offset;
		}
		else {
			// estimate the offset so that we don't stop whenever object is not found
			if ((m_target_bearing != DriveBase::kHeadingError) && (heading != DriveBase::kHeadingError)) {
				// estimate the angle from last measured tx and pigeon heading
				offset = m_target_bearing - heading;
			}
			else {
				// Stop as a last resort (pigeon not ready or failed)
				offset = 0;
			}
		}
		while (offset < -180.0)
			offset += 360.0;
		while (offset > 180)
			offset -= 360.0;

		// Start the rotate, and give it 100ms to settle
		if ((offset < -0.5) || (offset > 0.5)) {
			// Give motors a little power even if error is small
			if (offset > 0.0)
				rotation = kp*offset + minRotation;
			else
				rotation = kp*offset - minRotation;

			// Clip the maximum rotation for safety!
			if (rotation > maxRotation)
				rotation = maxRotation;
			if (rotation < -maxRotation)
				rotation = -maxRotation;

			std::cout << "Vision: offset " << offset << "rotation " << rotation << std::endl; //debug
			drive_base.ArcadeDriveForVision(0.0, -rotation);

			// give vision pipeline 40ms TBD: Does the motor keep turning? Does it stop at timeout?
			//m_timer = 2;
			//m_next_mode = kvision;
			//m_mode = ktimer;
			//m_mode = kvision;
		} else {
			m_mode = kdone;
			//m_timer = 4;	// wait 100ms then check again
			//m_next_mode = kvision_check;
			//m_mode = ktimer;
		}
	break;
	case kvision_check:
		if (object_found && ((tx < -0.5) || (tx > 0.5)))
			m_mode = kvision;
		else
			m_mode = kgrab_target;
	break;

	case kgrab_target:
		std::cout << "Vision: Ready to shoot\n"; //debug
		//frc::Joystick* joystick;
		// Just for fun. Not sure if it works!
		//joystick->SetRumble(frc::Joystick::kLeftRumble, 0.5);
		m_mode = kdone;
		break;
	case kdone:
	default:
		std::cout << "Vision: stopping\n"; //debug
		drive_base.Stop();
		break;
	}
}

bool VisionFindTarget::IsFinished() {
    // The command is finish when the done state is reached
    if (IsTimedOut()) {
        printf("VisionFindTarget command timed out\n");
        return true;
    }

    return m_mode == kdone;
}

void VisionFindTarget::End() {
    // Called once after isFinished returns true

	std::cout << "VisionFindTarget::End()\n";

    // Stop the drive base
    DriveBase::GetInstance().Stop();
}

void VisionFindTarget::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

	std::cout << "VisionFindTarget::Interrupted()\n";

    // Call the base class, which will just end the command
    Command::Interrupted();
}
