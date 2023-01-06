//==============================================================================
// PathfinderFollower.h
//==============================================================================

#pragma once

#include "PathFollower.h"
#include "../Pathfinder/PathfinderController.h"
#include <string>
class PathfinderController;
class PathSegment;
class RobotPath;
class Segment;

// PathfinderFollower is a adapter class that allows the new PathFollower code to
// use the old PathFinder code to follow a path. This code should eventually
// be obsolete when the new code does everything and performs better that the old.
class PathfinderFollower : public PathFollower {
public:
    
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	//
 	// robot_path - Path for the robot to follow. This object takes ownership.
	// drive_base - Drive base to control for following the path
   	// mechanism_controller - Mechanism controller for performing actions
	// record_samples - Whether samples are recorded and written to file for testing purposes
	// encoder_config - Configuration parameters for the Pathfinder code
	PathfinderFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
					   bool record_samples, const EncoderConfig& encoder_config);

	// Destructor
	virtual ~PathfinderFollower();

protected:
    //==========================================================================
    // Path Following (from PathFollower)

	virtual void StartPath();
	virtual void StartSegment();
	virtual void FollowSegment();
	virtual bool IsSegmentFinished();
	virtual void FinishSegment();
	virtual void FinishPath();

private:

    //==========================================================================
	// Nested Types

	// Sample data recorded during a test
	struct Sample
	{
		double m_time_s;				// Time in seconds the sample was taken at relative to the start of the test
		double m_left_distance_m;		// Measured left distance in metres, relative to path s
		double m_right_distance_m;		// Measured right error in metres
		double m_gyro_heading_deg;		// Measured gyro heading in degrees
		double m_left_output;			// Proportional output to the left motor [-1, 1]
		double m_right_output;			// Proportional output to the right motor [-1, 1]
		int m_segment_index;			// Index of the path segment being followed
		const std::string* m_mechanism_action; // Mechanism action performed (NULL for none)

		Segment m_left_trajectory;
		Segment m_right_trajectory;
	};


    //==========================================================================
	// Sample Recording

	// Setup the recording of 'sample' data for testing
	void SetupSampleRecording();

	// Record a test 'sample' for the current time period. Does nothing if 'm_record_samples' is false.
	//
   	// left_distance_m - Current distance travelled by the left side of the robot in metres
    // right_distance_m - Current distance travelled by the right side of the robot in metres
    // gyro_heading_deg - Current angle of the robot as read by the gyro in degrees
    // left_output - Returns the open loop drive in the range [-1.0, 1.0] for the left side of the robot
    // right_output - Returns the open loop drive in the range [-1.0, 1.0] for the right side of the robot
	// mechanism_action - Mechanism action performed this period, or NULL for none
	// left_segment - Path tranjectory segment for the left of the robot
	// right_segment - Path tranjectory segment for the right of the robot
	void RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
					  double left_output, double right_output, const std::string* mechanism_action,
					  Segment* left_segment, Segment* right_segment);

	// Write the recorded test samples to a file. Does nothing if 'm_record_samples' is false.
	void WriteTestSampleToFile();


    //==========================================================================
	// Member Variables

	EncoderConfig m_encoder_config;				// Configuration parameters for the Pathfinder code

    PathfinderController* m_path_controller;	// Path controller being used to follow the current segment
	double m_path_start_left_distance_m;		// Drivebase left wheel distance when the path starts
	double m_path_start_right_distance_m;		// Drivebase right wheel distance when the path starts

    std::vector<Sample> m_sample_list;			// List of data samples recorded during the test
	frc::Timer m_timer;							// Timer for measuring when samples occur
};
