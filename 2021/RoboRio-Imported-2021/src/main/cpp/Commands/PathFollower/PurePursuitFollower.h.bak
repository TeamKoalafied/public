//==============================================================================
// PurePursuitFollower.h
//==============================================================================

#pragma once

#include "PathFollower.h"

#include "../RobotPath/Point2D.h"
#include <frc/Timer.h>
#include <string>
#include <vector>

// TODO


// DONE
// Limiting velocity due to curvature means that the motion profile is ignored
// Need to calculate curvature to lookahead point
// do follower to lock to velocity
// Need to calculate lookahead point

// Calculating curvature
// https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
// https://en.wikiversity.org/wiki/CAGD/B%C3%A9zier_Curves



// PurePursuitFollower follows a path segment using the 'Pathfinder' method of calculating
// points for the left and right wheel and using a PID loop and heading adjustment to follow it.
class PurePursuitFollower : public PathFollower {
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

        double m_path_point_spacing;  // Spacing between generated path points
        double m_max_velocity;
        double m_max_acceleration;
        double m_max_velocity_curve;

        double m_lookahead_distance;
        double m_curvature_gain;
    };

    
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	//
 	// robot_path - Path for the robot to follow. This object takes ownership.
	// drive_base - Drive base to control for following the path
   	// mechanism_controller - Mechanism controller for performing actions
	// record_samples - Whether samples are recorded and written to file for testing purposes
	PurePursuitFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
					   bool record_samples);

    // Destructor
    virtual ~PurePursuitFollower();


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


    //==========================================================================
    // Testing

	// Generate the path points and write them to a file for testing
    //
    // filename - Path of the file to write to
    void TestGeneratePathToFile(const char* filename);

private:
    //==========================================================================
	// Private Nested Types

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
		Point2D m_robot_position;	    // Position of the robot
		Point2D m_closest_point;		// Closest point on the path
		Point2D m_lookahead_point;		// Lookahead point
        double m_curvature_lookahead;             // Curvature required to get to the lookahead point
        double m_curvature;             // Curvature required to get to the lookahead point
        double m_left_velocity;         // Desired left velocity
        double m_right_velocity;        // Desired right velocity
        double m_left_acceleration;     // Desired left acceleration
        double m_right_acceleration;    // Desired right acceleration
        double m_current_left_velocity; // Current left velocity
        double m_current_right_velocity;// Current right velocity
        int m_closest_index;                        // Index of the closest path point at the last update
        int m_lookahead_index;                      // Index of the lookahead path point at the last update
		const std::string* m_mechanism_action; // Mechanism action performed (NULL for none)
	};

    // Point on the linear approximate version of the path
    struct PathPoint
    {
        Point2D m_position;
        double m_heading_degrees;
        double m_distance;
        double m_velocity;
        double m_acceleration;
        double m_curvature;

        double m_max_velocity;
    };

    // State of a follow PID control (one is used for each side of the robot)
    struct FollowerState {
        double m_last_error;
        double m_total_error;
    };


    //==========================================================================
	// Pure Pursuit Calculations

    // Generate the list of PathPoints for the current segment
    void GenerateSegmentPathPoints();

    // Calculate the left and right motor outputs for the current instant
    //
    // left_output - returns the left motor output
    // right_output - returns the right motor output
    // sample - sample for recording details of the current update
    void CalculatePurePursuitDrive(double& left_output, double& right_output, Sample& sample);

    // Calculate the closest point on the path to the current robot position and the desired
    // velocity, acceleration and curvature. Only searches forward from the last point found.
    //
    // robot_position - Current robot position
    // path_velocity - Returns the target velocity for this point on the path
    // path_acceleration - Returns the target acceleration for this point on the path
    // path_curvature - Returns the target curvature for this point on the path
    //
    // Returns the closest point on the curve. If past the end of the path 'm_segment_finished'
    // will be set true and the final path point returned (final point of the actual path,
    // not the path 'extension')
    Point2D GetClosestPoint(Point2D robot_position, double& path_velocity, double& path_acceleration, double& path_curvature);

    // Calculate the lookahead point. Only searches forward from the last point found.
    //
    // robot_position - Current robot position
    // closest_point_distance - Current distance from the robot 
    //
    // Returns the lookahead position. If a lookahed point cannot be found 'm_segment_finished'
    // will be set true and the robot position returned.
   Point2D GetLookAheadPoint(Point2D robot_position, double closest_point_distance);


    //==========================================================================
    // Following Calculations

    // Do the PID follower calculation for one side of the robot
    //
    // follower_state - Follower state of this side of the robot
    // velocity - Target velocity of this side of the robot
    // acceleration - Target acceleration of this side of the robot
    // current_velocity - Current velocity of this side of the robot
    // feed_forward - Returns the feedforward compenent of the output
    //
    // Returns the new motor output in the range [-1,1] for this side of the robot
    double FollowSide(FollowerState& follower_state, double velocity, double acceleration, double current_velocity, double& feed_forward);


    //==========================================================================
	// Sample Recording

	// Setup the recording of 'sample' data for testing
	void SetupSampleRecording();

	// Write the recorded test samples to a file. Does nothing if 'm_record_samples' is false.
    //
    // filename - Path of the file to write ot
	void WriteTestSampleToFile(const char* filename);


	// Perform a linear interpolation between 2 scalar values
	//
	// pt1 - Start value. Corresponds to u == 0.0
	// pt2 - End value. Corresponds to u == 1.0
	// u - Interpolation parameter. Range is [0.0, 1.0] for interpolation
	//
	// Returns the interpolated value
	static double Lerp(double pt1, double pt2, double u) {
		return pt1 * (1.0 - u) + pt2 * u;
	}

    //==========================================================================
    // Member Variables

    FollowerParameters m_follower_parameters;   // Parameters that control how the path is followed

	double m_gyro_heading_offset_deg;			// Offset to add to the gyro to get the current heading in degrees
	double m_path_start_left_distance_m;		// Drivebase left wheel distance when the path started
	double m_path_start_right_distance_m;		// Drivebase right wheel distance when the path started
	double m_segment_start_left_distance_m;		// Drivebase left wheel distance when the current segment started
	double m_segment_start_right_distance_m;	// Drivebase right wheel distance when the current segment started

    FollowerState m_left_follower_state;        // State of the PID follower for the left side of the robot
    FollowerState m_right_follower_state;       // State of the PID follower for the right side of the robot

    std::vector<PathPoint> m_path_points;       // Point data that describes the current segment
    int m_closest_index;                        // Index of the closest path point at the last update
    int m_lookahead_index;                      // Index of the lookahead path point at the last update

    bool m_segment_finished;                    // Flag to indicate that the segment is finished
	int m_period_counter;                       // Counter of the update period
	int m_mechanism_actions_done_count;         // Count of how many mechanism actions have been done

    std::vector<PathPoint> m_total_path_points; // Point data that describes the whole path
    std::vector<Sample> m_sample_list;			// List of data samples recorded during the test
	frc::Timer m_timer;							// Timer for measuring when samples occur
};
