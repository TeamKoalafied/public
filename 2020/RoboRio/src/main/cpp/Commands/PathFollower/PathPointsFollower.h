//==============================================================================
// PathPointsFollower.h
//==============================================================================

#pragma once

#include "PathFollower.h"
#include "RobotPathPoints.h"
#include <frc/Timer.h>
#include <string>


// PathPointsFollower follows a path segment using the 'Pathfinder' method of calculating
// points for the left and right wheel and using a PID loop and heading adjustment to follow it.
class PathPointsFollower : public PathFollower {
public:
    //==========================================================================
    // Public Nested Types

    // Parameters that control how the path is generated and followed
    struct FollowerParameters {
        double m_kp;
        double m_ki;
        double m_kd;
        double m_kv;
        double m_kv_offset;
        double m_ka;
        double m_period_s;              // Update period in seconds
        double m_wheelbase_width_m;     // Width of the wheelbase in metres
    };

    
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	//
 	// robot_path - Path for the robot to follow. This object takes ownership.
	// drive_base - Drive base to control for following the path
   	// mechanism_controller - Mechanism controller for performing actions
	// record_samples - Whether samples are recorded and written to file for testing purposes
	PathPointsFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
					   bool record_samples);

    // Destructor
    virtual ~PathPointsFollower();


    //==========================================================================
    // Properties

    // Get the parameters that control the path following
    FollowerParameters& GetFollowerParameters() { return m_follower_parameters; }


    //==========================================================================
    // Path Following (from IPathFollower)

	virtual void StartPath();
	virtual void StartSegment();
	virtual void FollowSegment();
	virtual bool IsSegmentFinished();
	virtual void FinishSegment();
	virtual void FinishPath();

private:
    //==========================================================================
    // Private Nested Types

    // State of a follow PID control (one is used for each side of the robot)
    struct FollowerState {
        double m_last_error;
        double m_total_error;
    };


    //==========================================================================
    // Following Calculations

    // Do the PID follower calculation for one side of the robot
    //
    // follower_state - Follower state of this side of the robot
    // point - Path for this side of the robot
    // distance_m - Segment relative distance for this side of the robot
    // feed_forward - Returns the feedforward compenent of the output
    //
    // Returns the new motor output in the range [-1,1] for this side of the robot
    double FollowSide(FollowerState& follower_state, const RobotPathPoints::PathPoint& point, double distance_m, double& feed_forward);


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
		double m_left_feed_forward;		// Feedforward componenet of the output to the left motor
		double m_right_feed_forward;	// Feedforward componenet of the output to the right motor
		int m_segment_index;			// Index of the path segment being followed
		const std::string* m_mechanism_action; // Mechanism action performed (NULL for none)

		RobotPathPoints::PathPoint m_left_point;    // Path point for the left side of the robot
		RobotPathPoints::PathPoint m_right_point;   // Path point for the right side of the robot
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
	// left_feed_forward -  Feedforward componenet of the output to the left motor
	// right_feed_forward - Feedforward componenet of the output to the right motor
	// mechanism_action - Mechanism action performed this period, or NULL for none
	// left_point - Path point for the left side of the robot
	// right_point - Path point for the right side of the robot
	void RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
					  double left_output, double right_output, double left_feed_forward, double right_feed_forward,
                      const std::string* mechanism_action,
					  const RobotPathPoints::PathPoint& left_point, const RobotPathPoints::PathPoint& right_point);

	// Write the recorded test samples to a file. Does nothing if 'm_record_samples' is false.
	void WriteTestSampleToFile();


    //==========================================================================
    // Member Variables

    FollowerParameters m_follower_parameters;   // Parameters that control how the path is followed

    RobotPathPoints m_segment_points;           // Calculated path points for the segment
	double m_gyro_heading_offset_deg;			// Offset to add to the gyro to get the current heading in degrees
	double m_path_start_left_distance_m;		// Drivebase left wheel distance when the path started
	double m_path_start_right_distance_m;		// Drivebase right wheel distance when the path started
	double m_segment_start_left_distance_m;		// Drivebase left wheel distance when the current segment started
	double m_segment_start_right_distance_m;	// Drivebase right wheel distance when the current segment started

    FollowerState m_left_follower_state;        // State of the PID follower for the left side of the robot
    FollowerState m_right_follower_state;       // State of the PID follower for the right side of the robot

	int m_period_counter;                       // Counter of the update period
	int m_mechanism_actions_done_count;         // Count of how many mechanism actions have been done

    std::vector<Sample> m_sample_list;			// List of data samples recorded during the test
	frc::Timer m_timer;							// Timer for measuring when samples occur
};
