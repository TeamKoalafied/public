//==============================================================================
// PathSegment.h
//==============================================================================
#pragma once

#include "Bezier3.h"
#include "MotionProfile.h"
#include "MechanismAction.h"
#include <vector>
#include <string>

// A path segment represents the robot traveling in a single direction (forwards or
// backwards), along a path defined by Bezier curves, while performing actions
// with the robot's mechanisms.
class PathSegment
{
public:
	//==========================================================================
	// Construction

	PathSegment() {
		m_reverse = false;
	}

	//==========================================================================
	// Properties

	std::string m_name;						// Name of the segment. Used for logging and debugging.
	std::vector<Bezier3> m_path_definition; // List of Bezier curves that define the path to be followed.
	MotionProfile m_motion_profile;			// Motion profile controls how the robot moves along the path
	bool m_reverse;							// Whether the robot travels along the path in reverse
	std::vector<MechanismAction> m_mechanism_actions;
											// List of mechanism actions to perform
};

