//==============================================================================
// DrivePath.cpp
//==============================================================================
//
// DrivePath works by following a simple array of { distance traveled, required speed, required heading }
// The required heading is plus or minus degrees for each leg with respect to the initial heading before the command was run
// Each line shows the speed and heading requested up to the number of inches traveled.
// The heading is pidgeon corrected, the distance traveled is purely calculated by encoder count.
//
#include "DrivePath.h"


#include "../Subsystems/DriveBase.h"
#include "../Subsystems/Elevator.h"

#include <SmartDashboard/SmartDashboard.h>

#include <iostream>

//==============================================================================
// Construction

DrivePath::DrivePath(PathStrategy Strategy) :
	frc::Command("DrivePath") {

	switch (Strategy) {
	case PathStrategy::kSwitchLeft:
		m_lookup = SwitchLoadLeft_array;
		printf("DrivePath: Centre Pos, Left Switch\n");
		break;
	case PathStrategy::kSwitchRight:
		m_lookup = SwitchLoadRight_array;
		printf("DrivePath: Centre Pos, Right Switch\n");
		break;
	case PathStrategy::kLeftScaleLeft:
		m_lookup = LoadScaleLeftFromLeft_array;
		printf("DrivePath: Left Pos, Left Scale\n");
		break;
	case PathStrategy::kLeftScaleRight:
		m_lookup = LoadScaleRightFromLeft_array;
		printf("DrivePath: Left Pos, Right Scale\n");
		break;
	case PathStrategy::kRightScaleLeft:
		m_lookup = LoadScaleLeftFromRight_array;
		printf("DrivePath: Right Pos, Left Scale\n");
		break;
	case PathStrategy::kRightScaleRight:
		m_lookup = LoadScaleRightFromRight_array;
		printf("DrivePath: Right Pos, Right Scale\n");
		break;
	default:
	    m_lookup = TestDriveAndReturn_array;
		printf("DrivePath: Drive Forward\n");
		break;
	}

    m_target_distance = m_lookup[kPathArraySize-1].dist;

    std::cout << "DrivePath Distance,Angle " << m_target_distance << m_turn_angle_degrees << "\n";

    // Set the timeout for this command to something a bit longer than drive to scale and place and reverse
    const double kTimeoutS = 18.0;
    SetTimeout(kTimeoutS);
    mStop = false;
    m_distance_travelled = 0;
    m_last_reading = 0;

	// Driving requires the DriveBase
    Requires(&DriveBase::GetInstance());
    Requires(&Elevator::GetInstance());
    DriveBase::GetInstance().ResetDistance();
}


//==============================================================================
// Function Overrides from frc::Command

void DrivePath::Initialize() {
    // Called just before this Command runs the first time

	std::cout << "=============================================================================\n";
	std::cout << "DrivePath::Initialize()\n";
    std::cout << "Current Pigeon Heading: " << DriveBase::GetInstance().GetPigeonHeading() << "\n";

    m_finish_counter = 0;
    state = 0;

    // record encoder start position
    // m_start_point_inch = DriveBase::GetInstance().GetDistanceInch();
    // DriveBase::GetInstance().ResetDistance();
    m_distance_travelled = 0;
    m_last_reading = 0;

    // record the initial heading
    m_initial_heading_degrees = DriveBase::GetInstance().GetPigeonHeading();
    mStop = false;
	m_timer.Reset();
	m_timer.Start();

	Elevator::GetInstance().SetHighLiftSlowEnabled(false);
}

void DrivePath::Execute() {
    Elevator& elevator = Elevator::GetInstance();

    // Called repeatedly when this Command is scheduled to run

	// get encoder position, read heading and speed setting and feed to motor
	// m_distance_travelled = DriveBase::GetInstance().GetDistanceInch() - m_start_point_inch;
    int current_reading = DriveBase::GetInstance().GetDistanceInch();
    int diff = abs(current_reading - m_last_reading);
    m_last_reading = current_reading;

    m_distance_travelled += diff;

	// Get the current heading and calculate an error to our desired heading
	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    double error = m_target_heading_degrees - current_heading;

	// get required speed and heading for distance travelled
	for (int i=0; i<kPathArraySize; i++) {
		if (m_lookup[i].dist >= m_distance_travelled) {
			m_target_speed = m_lookup[i].speed;
			m_target_heading_degrees = m_lookup[i].heading + m_initial_heading_degrees;
			printf("Distance %d, Speed %d, Heading %f, diff %d\n",m_distance_travelled,m_target_speed,current_heading, diff);
			switch (m_lookup[i].IntakeCmd) {
				case IntakeControl::kNoCmd:
					break;
				case IntakeControl::kIntake:
					break;
				case IntakeControl::kEject:
					elevator.RunClawRollerEject(1,true);
					break;
				case IntakeControl::kRaiseFourBarLink:
					elevator.LiftArm();
					break;
				case IntakeControl::kLowerFourBarLink:
					elevator.DropArm();
					break;
				case IntakeControl::kMoveElevatorToFloor:
					elevator.MoveLiftToPosition(Elevator::LiftPosition::kFloor);
					break;
				case IntakeControl::kMoveElevatorToArmLift:
					elevator.MoveLiftToPosition(Elevator::LiftPosition::kArmLift);
					break;
				case IntakeControl::kMoveElevatorToSwitch:
					elevator.MoveLiftToPosition(Elevator::LiftPosition::kSwitch);
					break;
				case IntakeControl::kMoveElevatorToScale:
					elevator.MoveLiftToPosition(Elevator::LiftPosition::kScale);
					break;
				case IntakeControl::kMoveElevatorToIntermediate:
					elevator.MoveLiftToPosition(Elevator::LiftPosition::kIntermediate);
					break;
				case IntakeControl::kStop:
					mStop = true;
					printf("*** DrivePath Complete ***\n");
					break;
				default:
					break;
			}

			break;
		}
	}


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
    		//reduce turn gain if below 80 if rotating faster than 180 dps
    		if (((fabs(error)<80)&&(fabs(currentAngularRate)>120))||(state == 1)) {
    			kRotateGain = 0.005;
    			if (state!=1) {
    				state = 1;
    				//printf("60 degree: %f, error %f\n",currentAngularRate,error);
    			}
    		}
    		//reduce turn gain below 10
    		if (fabs(error) < 10) {
    			kRotateGain = 0.001;
    			if (state!=2) {
    				state = 2;
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
    DriveBase::GetInstance().Drive(m_target_speed, rotate);
}

bool DrivePath::IsFinished() {
    // Make this return true when this Command no longer needs to run execute()

	SmartDashboard::PutNumber("PathDistance", DriveBase::GetInstance().GetDistanceInch());

    if (IsTimedOut()) {
        printf("DrivePath command timed out\n");
        return true;
    }

    return mStop;
}

void DrivePath::End() {
    // Called once after isFinished returns true

	// Log the target heading, the current heading and the error between them
	double current_heading = DriveBase::GetInstance().GetPigeonHeading();
    printf("DrivePath::End() target heading %f  current heading %f  error %f time %f\n", m_target_heading_degrees,
    		current_heading, m_target_heading_degrees - current_heading, m_timer.Get());

    // Stop the drive base
    DriveBase::GetInstance().Stop();

	Elevator::GetInstance().SetHighLiftSlowEnabled(true);
}

void DrivePath::Interrupted() {
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

    // Call the base class, which will just end the command
    Command::Interrupted();
}
