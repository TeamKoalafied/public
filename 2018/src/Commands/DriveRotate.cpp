//==============================================================================
// DriveRotate.cpp
//==============================================================================

#include "DriveRotate.h"

#include "../Subsystems/DriveBase.h"

#include <SmartDashboard/SmartDashboard.h>

#include <iostream>

//==============================================================================
// Construction

DriveRotate::DriveRotate(double turn_angle_degrees) :
	frc::Command("DriveRotate") {
    m_turn_angle_degrees = turn_angle_degrees;

    std::cout << "DriveRotate Angle " << m_turn_angle_degrees << "\n";

    // Set the timeout for this command to something a bit longer than a full 360
    // degree turn would take
    // TODO: Hack to 5s for now
    const double kTimeoutS = 5.0;
    SetTimeout(kTimeoutS);

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());
}


//==============================================================================
// Function Overrides from frc::Command

void DriveRotate::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "=============================================================================\n";
	std::cout << "DriveRotate::Initialize()\n";
    std::cout << "Current Pigeon Heading: " << DriveBase::GetInstance().GetPigeonHeading() << "\n";

    // Calculate the heading to turn to
    m_target_heading_degrees = DriveBase::GetInstance().GetPigeonHeading() + m_turn_angle_degrees;
    m_finish_counter = 0;
    state = 0;

	m_timer.Reset();
	m_timer.Start();
}

void DriveRotate::Execute() {
    // Called repeatedly when this Command is scheduled to run


	// Get the current heading and calculate an error to our desired heading
	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    double error = m_target_heading_degrees - current_heading;

    // get rotation speed in degrees per second
	double gyro_xyz_dps[3];
	DriveBase::GetInstance().GetPigeonRawGyro(gyro_xyz_dps);
	double currentAngularRate = gyro_xyz_dps[2];

	// send current heading adjustment still needed and current rotation speed to dashboard
    SmartDashboard::PutNumber("HError", error);
    SmartDashboard::PutNumber("Gyro", currentAngularRate);

    // Calculate a rotate drive value to turn the robot towards the desired heading
    double kRotateGain = SmartDashboard::GetNumber("Rotate Gain", 0.005);

    	// if down to between 60 and 20 degrees from required turn and still rotating faster than 210 degrees per second apply brakes !
    if ((fabs(error)<60) && (fabs(error)>20) && (fabs(currentAngularRate)>210)) {
    		kRotateGain *= 1;   // brake effect removed for now
    		printf("Brake on: %f, error %f\n",currentAngularRate,error);
    }
    else {
    		// reduce turn gain below 180 and above 100 degrees
    		if ((fabs(error)<180)&&(fabs(error)>100)) {
    				kRotateGain = 0.004;
    		}

    		// reduce turn gain if below 100 degrees and rotating faster than 220 dps
    		if (((fabs(error)<100) && (fabs(currentAngularRate)>220)) || (state == 2)) {
    			kRotateGain = 0.004;
    			if (state != 2) {
    				state = 2;
    				//printf("90 degree: %f, error %f\n",currentAngularRate,error);
    			}
    		}
    		//reduce turn gain if below 80 if rotating faster than 180 dps
    		if (((fabs(error)<80)&&(fabs(currentAngularRate)>120))||(state == 3)) {
    			kRotateGain = 0.004;
    			if (state!=3) {
    				state = 3;
    				//printf("60 degree: %f, error %f\n",currentAngularRate,error);
    			}
    		}
    		//reduce turn gain below 10
    		if (fabs(error) < 10) {
    			kRotateGain = 0.001;
    			if (	state!=4) {
    				state = 4;
    				//printf("10 degree: %f, error %f\n",currentAngularRate,error);
    			}
    		}
    }

    double rotate = error * kRotateGain;

    // Clip the rotate to the allowed range [-1, 1] and update the drive base
    if (rotate > 1.0) rotate = 1.0;
    if (rotate < -1.0) rotate = -1.0;

    // ensure minimum drive strength to actually turn robot
    if (rotate > 0) {
        if (rotate < 0.19) rotate = 0.19;
    }else{
       if (rotate > -0.19) rotate = -0.19;
    }

    // if within 3 degrees of required heading, stop
    if (fabs(error) < 3) {
    		rotate = 0.0;
    }

    SmartDashboard::PutNumber("Rotate", rotate);
    SmartDashboard::PutNumber("RError", error);

    // apply rotate drive current to drive base
    DriveBase::GetInstance().Drive(0.0, rotate);
}

bool DriveRotate::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

	SmartDashboard::PutNumber("DDistance", DriveBase::GetInstance().GetDistanceInch());

    if (IsTimedOut()) {
        printf("DriveRotate command timed out\n");
        return true;
    }

	// Get the current heading and calculate an error to our desired heading
	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    double error = m_target_heading_degrees - current_heading;

    // The command is done if the error is below a threshold margin
    const double kRotateMargin = 3.0;
    if (fabs(error) < kRotateMargin) {
        m_finish_counter++;
    }else {
        m_finish_counter = 0;
    }

    return m_finish_counter >= 20;
}

void DriveRotate::End() {
    // Called once after isFinished returns true

	// Log the target heading, the current heading and the error between them
	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    printf("DriveRotate::End() target heading %f  current heading %f  error %f time %f\n", m_target_heading_degrees,
    		current_heading, m_target_heading_degrees - current_heading, m_timer.Get());

    // Stop the drive base
    DriveBase::GetInstance().Stop();
}

void DriveRotate::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}
