//==============================================================================
// DriveStraight.cpp
//==============================================================================

#include <Commands/DriveStraight.h>
#include "../Subsystems/DriveBase.h"

#include <SmartDashboard/SmartDashboard.h>

#include <iostream>

//==============================================================================
// Construction

DriveStraight::DriveStraight(double velocity_feet_per_second, double distance_inch, double heading) :
	frc::Command("DriveAutonomous") {

	// assign local member variables
	m_velocity_feet_per_second = velocity_feet_per_second;
    m_distance_inch = distance_inch;
    m_heading = heading;

	std::cout << "DriveStraight Velocity: " << m_velocity_feet_per_second <<
			     " Distance: " << m_distance_inch <<  " Heading: " << m_heading << "\n";

    SetupTimeout();

    // set baking mode to false from start
    m_braking = false;

    // set the required braking distance depending on the total distance to travel as speed when brake is applied will vary with distance
    if (distance_inch > 55) {
   		m_braking_distance = 21;
    } else {
		if (distance_inch > 48){
			m_braking_distance = 21;
		} else {
			if (distance_inch > 35){
				m_braking_distance = 18;
			} else {
				if (distance_inch > 24){
					m_braking_distance = 13;
				} else {
					if (distance_inch <= 24){
						m_braking_distance = 13;
					}
				}
			}
		}
    }

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void DriveStraight::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "=============================================================================\n";
	std::cout << "DriveStraight::Initialize()\n";
    std::cout << "Current Pigeon Heading: " << DriveBase::GetInstance().GetPigeonHeading() << "\n";
    std::cout << "Velocity: " << m_velocity_feet_per_second << "\n";
    std::cout << "Distance: " << m_distance_inch << "\n";
    std::cout << "Heading Parameter: " << m_heading << "\n";

    // Calculate the target encoder distance position to drive to
    m_target_distance_inch = DriveBase::GetInstance().GetDistanceInch() + m_distance_inch;
    std::cout << "Target Distance: " << m_target_distance_inch << "\n";

    // Tell the drivebase to drive straight on the heading we want
    DriveBase::GetInstance().StartDrivingStraight(m_heading);

    // Clear the braking flag
    m_braking = false;

    // Start the timer
	m_timer.Reset();
	m_timer.Start();
}

void DriveStraight::Execute() {
    // Called repeatedly when this Command is scheduled to run

	// Calculate the distance remaining to our target position
	DriveBase& drive_base = DriveBase::GetInstance();
	double distance_travelled_inch = drive_base.GetDistanceInch();
	double distance_remaining_inch = m_target_distance_inch - distance_travelled_inch;

	// If the distance is less that the braking distance, then start braking
	if (fabs(distance_remaining_inch) < m_braking_distance) {
		if (!m_braking) {
			m_braking = true;
			printf("Started braking %f\n",distance_remaining_inch);
		}
	}

	// If breaking then stop, else we drive at the set speed.
	if (m_braking) {
		// stop
		drive_base.Stop();
	} else {
		// keep going, not yet at braking distance
		drive_base.Drive(m_velocity_feet_per_second, 0.0);
	}
}

bool DriveStraight::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()
	DriveBase& drive_base = DriveBase::GetInstance();
    if (IsTimedOut()) {
        printf("DriveStraight command timed out\n");
        return true;
    }

    // The command is complete if we are braking and the velocity is near zero
    if ((m_braking) && (fabs(drive_base.GetVelocityFeetPerSecond()) < 0.3)) {
   		return true;
    } else {
   		return false;
    }
}

void DriveStraight::End() {
    // Called once after isFinished returns true

    printf("DriveStraight::End() distance %f  velocity %f time %fs\n",
    		DriveBase::GetInstance().GetDistanceInch(), DriveBase::GetInstance().GetVelocityFeetPerSecond(),
			m_timer.Get());

    DriveBase::GetInstance().Stop();
}

void DriveStraight::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}


//==========================================================================
// Operations

void DriveStraight::Set(double velocity_feet_per_second, double distance_inch, double heading) {

	// what calls this function ?

	// set member vars
	m_velocity_feet_per_second = velocity_feet_per_second;
	m_distance_inch = distance_inch;
	m_heading = heading;
	m_braking = false;

	SetupTimeout();
    if (distance_inch > 40) {
    	    m_braking_distance = 24;
    } else {
    	    m_braking_distance = 21;
    }
}


//==========================================================================
// Timeout Calculation

void DriveStraight::SetupTimeout() {
    // Set the timeout for this command to something a bit longer than it should take.
    double timeout_s = 2.0*(m_distance_inch/12.0)/m_velocity_feet_per_second + 2.0;
    printf("timeout_s %f\n", timeout_s);
    SetTimeout(timeout_s);
}
