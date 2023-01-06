//==============================================================================
// PathFollower.h
//==============================================================================

#pragma once

class IMechanismController;
class IPathDriveBase;
class RobotPath;
class PathSegment;

// PathFollower is a base class for controllers that follow a RobotPath.
// This class handles following each of the PathSegments in the RobotPath in turn.
// In particular, it deals with using dead reckoning so that the when one path segment
// does not finish in the exact correct location the following path is adjusted to
// account for it. Classes derived from this one handle following each PathSegment.
class PathFollower {
public:
   
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	//
	// robot_path - Path for the robot to follow
	// drive_base - Drive base to control for following the path
   	// mechanism_controller - Mechanism controller for performing actions
	// record_samples - Whether samples are recorded and written to file for testing purposes
	PathFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
						   bool record_samples);

	// Destructor
	virtual ~PathFollower();


    //==========================================================================
	// Path Following

	// Initialise the controller for following the path
	virtual void Start();

	// Follow the path for the current time period
	//
	// drive_base - Drivebase used to follow the path
	virtual void Follow();

	// Get whether the controller is finished following the path
	virtual bool IsTrajectoryFinished();

	// Finish the path. May be called before the path finishes if following it is cancelled.
	virtual void Finish();

protected:
    //==========================================================================
	// Properties

	// Get the path for the robot to follow
	const RobotPath* GetRobotPath() const { return m_robot_path; } 

	// Get the drive base to control for following the path
	IPathDriveBase* GetDriveBase() { return m_drive_base; }
   	
	// Get the mechanism controller for performing actions
	IMechanismController* GetMechanismController() { return m_mechanism_controller; }

	// Get whether samples are recorded and written to file for testing purposes
	bool GetRecordSamples() { return m_record_samples; }

	// Get the current segment being followed (valid when command is running)
	PathSegment& GetPathSegment();

	// Get the index of the current segment being followed (valid when command is running)
	int GetPathSegmentIndex() { return m_path_segment_index; }

	// Get whether the path has actually finished
	bool GetPathFinished() { return m_path_finished; }


    //==========================================================================
    // Path Following

    // Start following a robot path
	virtual void StartPath() = 0;

    // Start following a path segment within the overall robot path
	virtual void StartSegment() = 0;

    // Follow the current path segment. Updates the motor outputs.
	virtual void FollowSegment() = 0;

    // Test if the current path segment is complete (including mechanism actions)
    //
    // Return whether the path segment has been completed
    virtual bool IsSegmentFinished() = 0;

    // Finish following the current path segment
	virtual void FinishSegment() = 0;

    // Finish following the robot path
	virtual void FinishPath() = 0;

private:
    //==========================================================================
	// Path Segment Setup

	// Setup the next path segment to be followed
	void SetupPathSegment();

	// Setup a path segment to start at the current robot position
	void AdjustPathSegmentStart(PathSegment& path_segment);


    //==========================================================================
    // Member Variables

	RobotPath* m_robot_path;					// Path for the robot to follow	
	IPathDriveBase* m_drive_base;				// Drive base to control for following the path
   	IMechanismController* m_mechanism_controller;   // Mechanism controller for performing actions
	bool m_record_samples;						// Whether samples are recorded and written to file for testing purposes

	int m_path_segment_index;					// Index of the current segment being followed (valid when command is running)
	bool m_path_finished;						// Whether the path has actually finished
};

