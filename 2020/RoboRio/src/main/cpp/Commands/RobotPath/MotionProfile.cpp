//==============================================================================
// MotionProfile.cpp
//==============================================================================

#include "MotionProfile.h"

#include <math.h>
#include <iostream>

//==========================================================================
// Construction

MotionProfile::MotionProfile()
{
}


//==========================================================================
// Initialisation


void MotionProfile::Setup(double max_velocity, double max_acceleration)
{
	m_max_velocity = max_velocity;
	m_max_acceleration = max_acceleration;
}

void MotionProfile::InitialiseProfile(double length)
{
	// Record the length of the path the motion profile is for
	m_length = length;

	// Calculate the time and distance spent accelerating, assuming maximum velocity can be reached
	m_acceleration_time = m_max_velocity / m_max_acceleration;
	m_acceleration_distance = 0.5 * m_max_acceleration * m_acceleration_time * m_acceleration_time;

	// If reaching maximum velocity takes more that the length of the path adjust the acceleration
	// time and distance
	if (m_acceleration_distance > m_length / 2.0) {
		m_acceleration_distance = m_length / 2.0;
		m_acceleration_time = sqrt(2.0 * m_acceleration_distance / m_max_acceleration);
	}

	// Calculate the total time the path will take
	double max_velocity_distance = m_length - 2.0 * m_acceleration_distance;
	double max_velocity_time = max_velocity_distance / m_max_velocity;
	m_total_time = 2.0 * m_acceleration_time + max_velocity_time;
}


//==========================================================================
// Calculations

void MotionProfile::GetMotion(double time, double& position, double& velocity, double& acceleration) const
{
	if (time < m_acceleration_time) {
		// Time is within the initial acceleration period
		position = 0.5 * m_max_acceleration * time * time;
		velocity = m_max_acceleration * time;
		acceleration = m_max_acceleration;
	}
	else if (time < m_total_time - m_acceleration_time) {
		// Time is within the constant velocity period
		position = m_max_velocity * (time - m_acceleration_time) + m_acceleration_distance;
		velocity = m_max_velocity;
		acceleration = 0.0;
	}
	else if (time < m_total_time) {
		// Time is within the final deceleration period
		time = m_total_time - time;
		position = m_length - 0.5 * m_max_acceleration * time * time;
		velocity = m_max_acceleration * time;
		acceleration = -m_max_acceleration;
	}
	else {
		// Time is after the profile is complete
		position = m_length;
		velocity = 0.0;
		acceleration = 0.0;
	}
}
