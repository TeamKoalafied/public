//==============================================================================
// VisionFindCube.cpp
//==============================================================================

#include "VisionFindCube.h"

#include "../Subsystems/DriveBase.h"
#include "../Subsystems/VisionSystem.h"
#include "../Subsystems/Elevator.h"

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

    // Debug data. This loop runs 5 times (20ms) for every camera grab (100ms)
    if (m_test_counter == 0) {
		std::cout << "found = " << object_found;
		std::cout << " dist = " << distance_to_object;
		std::cout << " dir = " << direction_of_object << "\n";
    }
    m_test_counter = (m_test_counter + 1) % 5;

    // Tell the robot to SLOWLY turn left or right depending on the sign and size of the offset
    //myRobot->Drive(-0.1, -0.5 * globalOffset);
    switch (m_mode) {
    case killuminate_scene :
    	m_relay.Set(Relay::kReverse); //mhm turn flood light on
    	m_mode = kfind_cube;
        m_old_direction_of_object = 0.3;
        if (Elevator::GetInstance().IsArmUp())
        	Elevator::GetInstance().DropArm();
        if (!Elevator::GetInstance().IsClawOpen())
        	Elevator::GetInstance().OpenClaw();
    	break;
    case kfind_cube :
        if (object_found) {
#if 0
        	//debug
            //end of debug
            double speed = 0.25;
            //max_spins = 0;
            if (direction_of_object > 1.0)
                direction_of_object = 1.0;
            if (direction_of_object < -1.0)
                direction_of_object = -1.0;

            if (distance_to_object > 1.0)
                distance_to_object = 1.0;

            // Stop when box is 1 row from bottom of screen. 1/120 = 0.0083333.
            if (distance_to_object < 0.5)
                speed = 0.25;
            if (distance_to_object < 0.02){
                speed = 0.0;
            m_mode = kgrab_cube; //mhm debug
            }
            m_old_direction_of_object = direction_of_object;

            if ((direction_of_object < 0.1) && (direction_of_object > -0.1) && (distance_to_object <= 0.02)){//if the max height of the object is within the bottom 3 rows the cube will be intaken
                drive_base.ArcadeDriveForVision(0, 0);
                std::cout << "Found the cube\n";
                m_mode = kgrab_cube; //mhm
            }
            else {
                //myRobot->ArcadeDrive(0.5 * distance_to_object, 0.3 * direction_of_object);
                drive_base.ArcadeDriveForVision(speed_factor * speed, rotate_factor * direction_of_object); //speed_constant is for debug
            }
#else
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
#endif
        }

        else {
        	double direction;

        	if ((m_old_direction_of_object <= -0.3) || (m_old_direction_of_object >= 0.3)) { //max changed 0.2 to 0.3
                m_old_direction_of_object = m_old_direction_of_object * 0.9;
                direction = m_old_direction_of_object;
            } else {
                if (m_old_direction_of_object < 0.0)
                	direction = -0.3;
                else
                	direction = 0.3;
            }
            drive_base.ArcadeDriveForVision(0, rotate_factor * direction);
        }
        break;
    case kgrab_cube:

        //grab cube
        std::cout << "Ready to grab!\n";
        m_relay.Set(Relay::kForward); //mhm debug
        m_mode = kdone; //Debug should finish
        break;
    case kdone:
    	m_relay.Set(Relay::kForward); //mhm
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
	m_relay.Set(Relay::kReverse); //mhm
}
void VisionFindCube::TurnOffFloodlight() {
	m_relay.Set(Relay::kForward); //mhm
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
