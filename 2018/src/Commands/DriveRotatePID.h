//==============================================================================
// DriveRotatePID.h
//==============================================================================

#ifndef DriveRotatePID_H
#define DriveRotatePID_H

#include <PIDController.h>
#include <PIDOutput.h>
#include <PIDSource.h>
#include <Commands/Command.h>


// Command for rotating given angle on the spot
class DriveRotatePID : public frc::Command, private frc::PIDSource, private frc::PIDOutput {
public:
	// Whether the angle is relative or absolute
	enum class AngleType {
		kRelative,
		kAbsolute
	};

    //==========================================================================
    // Construction

    // Constructor
	//
	// angle_type - Whether the turn angle is relative or absolute
    // turn_angle_degrees - Turn angle in degrees
	DriveRotatePID(AngleType angle_type, double turn_angle_degrees);

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
    //==========================================================================

	// Set the turn angle
	//
	// angle_type - Whether the turn angle is relative or absolute
    // turn_angle_degrees - Turn angle in degrees
	void SetTurnAngle(AngleType angle_type, double turn_angle_degrees) {
		m_angle_type = angle_type;
		m_turn_angle_degrees = turn_angle_degrees;
	}

private:
    //==========================================================================
	// Function Overrides from frc::PIDSource
	virtual double PIDGet() override;
    //==========================================================================

    //==========================================================================
	// Function Overrides from frc::PIDOutput
	virtual void PIDWrite(double output) override;
    //==========================================================================

    //==========================================================================
	// Member Variables

	AngleType m_angle_type;				// Whether the turn angle is relative or absolute
	double m_turn_angle_degrees;		// Turn angle in degrees

	double m_target_heading_degrees;	// Heading angle to turn to for the current execution in degrees
	int m_finish_counter;					// Counter for being on the required angle long enough to stop

	frc::PIDController* m_pid_controller;	// PID controller for rotation
};

#endif  // DriveRotate_H
