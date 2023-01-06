//==============================================================================
// MechanismAction.h
//==============================================================================
#pragma once

#include <string>

// A mechanism action is a single action performed by one of the robot's mechanisms
// during autonomous path following.
struct MechanismAction {
	enum TimeSpecification {
		// NICKTODO
		Start,			// Time is relative to the start of the path segment in seconds
		End,			// Time is relative to the end of the path segment in seconds
	};


	std::string m_action;					// Name of the action to perform
	TimeSpecification m_time_specification; // How the time is specified
	double m_time;							// Value that specifies the time
	int m_time_period;						// Index of the time period to perform the action in (calculate when path is running)
};


