//==============================================================================
// VisionFindCube.cpp
//==============================================================================

#include "VisionFindCube.h"

#include "../Subsystems/DriveBase.h"
#include "../Subsystems/VisionSystem.h"

#include <iostream>



//==============================================================================
// Construction

VisionFindCube::VisionFindCube() :
    frc::Command("VisionFindCube", 15.0) {

    // This command requires the vision system and the drive base
    Requires(&DriveBase::GetInstance());
    Requires(&VisionSystem::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void VisionFindCube::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "VisionFindCube::Initialize()\n";

    m_mode = killuminate_scene;
    m_old_direction_of_object = 0.3;
    m_test_counter = 0;
    VisionSystem::GetInstance().SetDarkImage(true);
}

void VisionFindCube::Execute() {
    // Called repeatedly when this Command is scheduled to run
	std::cout << "the current mode is " << m_mode << std::endl; //debug
    // Get the current state of finding the object from the vision system
    double distance_to_object = 0.0; //Distance from bottom of image 0.0-1.0
    double direction_of_object = 0.0; // Direction of object -1.0 to 1.0
    bool object_found = true; // Whether or not the vision code found the object
    object_found = VisionSystem::GetInstance().GetCubeFound(distance_to_object, direction_of_object);

    DriveBase& drive_base = DriveBase::GetInstance();

    const double speed_factor = 0.6; //was 1
    const double rotate_factor = -0.6; //was -0.6 and 0.8

    // Tell the robot to SLOWLY turn left or right depending on the sign and size of the offset
    //myRobot->Drive(-0.1, -0.5 * globalOffset);
    switch (m_mode) {
    case killuminate_scene :
    	m_relay.Set(frc::Relay::kReverse); //mhm turn flood light on
    	m_mode = kfind_cube;
        m_old_direction_of_object = 0.3;
        //if (Elevator::GetInstance().IsArmUp())
       // 	Elevator::GetInstance().DropArm();
        //if (!Elevator::GetInstance().IsClawOpen())
        //	Elevator::GetInstance().OpenClaw();
        //Elevator::GetInstance().MoveLiftToPosition(Elevator::LiftPosition::kFloor); // 4" lift?
    	break;
    case kfind_cube :
        if (object_found) {
            m_test_counter = 5;
            m_old_direction_of_object = direction_of_object;
            // If the bottom of the object is within the bottom 3 rows the cube is close enough
            if (distance_to_object <= 0.02) {
                drive_base.ArcadeDriveForVision(0, 0);
                std::cout << "Found the cube\n";
                m_mode = kgrab_cube; //mhm
            }
            else {
            	drive_base.ArcadeDriveForVision(
            			speed_factor * GetSpeed(distance_to_object, direction_of_object),
						rotate_factor * GetRotation(distance_to_object, direction_of_object)
            	);
            }
        }

        else {
            // If we cannot see the cube for 100ms then stop (safer than spinning and finding a non-cube)
            if (m_test_counter <= 0) {
            	std::cout << "Could not find the cube. Aborting!\n";
                m_mode = kdone;
            }
            else {
            	m_test_counter--;
            }
        }
        break;
    case kgrab_cube:

        //grab cube
        std::cout << "Ready to grab!\n";
        //Elevator::GetInstance().CloseClaw();
        //Elevator::GetInstance().RunClawRollerGrab(1);  // 1 second grab?
        //Elevator::GetInstance().MoveLiftToPosition(Elevator::LiftPosition::kArmLift); // 4" lift?
    	//Elevator::GetInstance().LiftArm();
        m_relay.Set(frc::Relay::kForward); //mhm debug
        m_mode = kdone; //Debug should finish
        break;
    case kdone:
    	m_relay.Set(frc::Relay::kForward); //mhm
    default:
        break;
    }
}

bool VisionFindCube::IsFinished() {
    // The command is finish when the done state is reached
    if (IsTimedOut()) {
        printf("VisionFindCube command timed out\n");
        return true;
    }

    return m_mode == kdone;
}

void VisionFindCube::End() {
    // Called once after isFinished returns true

	std::cout << "VisionFindCube::End()\n";

	// Restore the camera to normal settings
	VisionSystem::GetInstance().SetDarkImage(false);

    // Stop the drive base
    DriveBase::GetInstance().Stop();
}

void VisionFindCube::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

	std::cout << "VisionFindCube::Interrupted()\n";

    // Call the base class, which will just end the command
    Command::Interrupted();
}

void VisionFindCube::TurnOnFloodlight() {
	m_relay.Set(frc::Relay::kReverse); //mhm
}
void VisionFindCube::TurnOffFloodlight() {
	m_relay.Set(frc::Relay::kForward); //mhm
}

double VisionFindCube::GetSpeed(double distance, double angle) {
	const double speed[5][5] = {
		//          distance
		//   0.2, 0.4, 0.6, 0.8, 1.0
			{0.2, 0.4, 0.8, 1.0, 1.0},  // 0.0 < angle < 0.2
			{0.0, 0.2, 0.5, 0.7, 0.7},  // 0.2 < angle < 0.4
			{0.0, 0.2, 0.2, 0.4, 0.5},  // 0.4 < angle < 0.6
			{0.0, 0.2, 0.2, 0.3, 0.4},  // 0.6 < angle < 0.8
			{0.0, 0.2, 0.2, 0.2, 0.3}   // 0.8 < angle < 1.0
	};
	double result;
	if (angle >= 0.0) {
		int a = (int)(angle * 4.9);
		int d = (int)(distance * 4.9);
		if (a > 4)
			a = 4;
		if (d > 4)
			d = 4;
		result = speed[a][d];
	}
	else {
		int a = (int)(angle * -4.9);
		int d = (int)(distance * 4.9);
		if (a > 4)
			a = 4;
		if (d > 4)
			d = 4;
		result = speed[a][d];
	}
	if (distance < 0.02)
		result = 0.0;

//	result = 0.0; // to tune the rotation
	return result;
}

double VisionFindCube::GetRotation(double distance, double angle) {
	const double rotation[5][5] = {
		//          distance
		//   0.2, 0.4, 0.6, 0.8, 1.0
			{0.1, 0.1, 0.1, 0.1, 0.1},  // 0.0 < angle < 0.2
			{0.2, 0.2, 0.2, 0.2, 0.2},  // 0.2 < angle < 0.4
			{0.2, 0.2, 0.2, 0.2, 0.2},  // 0.4 < angle < 0.6
			{0.3, 0.3, 0.3, 0.3, 0.3},  // 0.6 < angle < 0.8
			{0.4, 0.4, 0.4, 0.4, 0.4}   // 0.8 < angle < 1.0
	};
	double result;
	if (angle >= 0.0) {
		int a = (int)(angle * 4.9);
		int d = (int)(distance * 4.9);
		if (a > 4)
			a = 4;
		if (d > 4)
			d = 4;
		result = rotation[a][d];
	}
	else {
		int a = (int)(angle * -4.9);
		int d = (int)(distance * 4.9);
		if (a > 4)
			a = 4;
		if (d > 4)
			d = 4;
		result = -rotation[a][d];
	}
	return result;
}
