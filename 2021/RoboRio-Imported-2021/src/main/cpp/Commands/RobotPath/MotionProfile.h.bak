//==============================================================================
// MotionProfile.h
//==============================================================================
#pragma once

// A motion profile represent how far the along the path the robot should be at any
// time taking into acount the desired velocity and acceleration.
// Any distance and time units can be used as long as they are consistent.
class MotionProfile
{
public:
	//==========================================================================
	// Construction

	// Default constructor
	MotionProfile();

	//==========================================================================
	// Initialisation

	// Setup the velocity and acceleration
	//
	// max_velocity - Maximum velocity for the profile
	// max_acceleration - Maximum aceleration for the profile
	void Setup(double max_velocity, double max_acceleration);

	// Initialise the profile for a path of a certain length
	//
	// length - Length of the path the motion profile is being used with
	void InitialiseProfile(double length);

	//==========================================================================
	// Calculations

	// Get the motion parameters at given time
	//
	// time - Time to get the parameters for
	// position - Returns the position of the robot along the path
	// velocity - Returns the velocity of the robot along the path
	// acceleration - Returns the acceleration of the robot along the path
	void GetMotion(double time, double& position, double& velocity, double& acceleration) const;

	// Get the total time the path will take to complete
	double GetTotalTime() const { return m_total_time; }

private:
	//==========================================================================
	// Member Variables

    double m_max_velocity;				// Maximum velocity for the profile
	double m_max_acceleration;			// Maximum aceleration for the profile

	double m_length;					// Total length of the profile
	double m_acceleration_distance;		// Distance that acceleration occurs over at the beginning and end of the profile
	double m_acceleration_time;			// Time that acceleration occurs over at the beginning and end of the profile
	double m_total_time;				// Total time taked to travel along the profile
};

