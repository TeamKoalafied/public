//==============================================================================
// RobotPath.h
//==============================================================================
#pragma once

#include "PathSegment.h"
#include <vector>
#include <string>

// A robot path represents the robot moving along a complex path, including possibly
// changing direction, while performing actions with the robot's mechanisms.
// The path is defined by a list of path segments.
class RobotPath
{
public:
	//==========================================================================
	// Construction

	RobotPath() {
	}

	~RobotPath() {
		for (auto p : m_path_segments) {
			delete p;
		} 
		m_path_segments.clear();
	}

	//==========================================================================
	// Properties

	std::string m_name;						    // Name of the path. Used for logging and debugging.
	std::vector<PathSegment*> m_path_segments;  // List of path segments that define the path to be followed. This class owns the PathSegment objects.
};
