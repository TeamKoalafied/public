//==============================================================================
// PathPointsFollower.h
//==============================================================================

#include "PathPointsFollower.h"

#include "RobotPathPoints.h"
#include "../RobotPath/IMechanismController.h"
#include "../RobotPath/IPathDriveBase.h"
#include "../RobotPath/PathSegment.h"
#include "../RobotPath/RobotPath.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>

//==========================================================================
// Construction and Destruction

PathPointsFollower::PathPointsFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
					   				   bool record_samples) :
	PathFollower(robot_path, drive_base, mechanism_controller, record_samples) {

    // Setup sensible default values for the parameters
   	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
 	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V
    m_follower_parameters.m_kp = 1.0;
    m_follower_parameters.m_ki = 0.0;
    m_follower_parameters.m_kd = 0.0;
    m_follower_parameters.m_kv = 1.0/4.28;
    m_follower_parameters.m_kv_offset = 0.104;
    m_follower_parameters.m_ka =  1.0/18.29;
    m_follower_parameters.m_period_s = 0.02; // 20ms
    m_follower_parameters.m_wheelbase_width_m = 0.71;
}

PathPointsFollower::~PathPointsFollower() {
}


//==========================================================================
// Path Following (from IPathFollower)

void PathPointsFollower::StartPath() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);

    m_path_start_left_distance_m = left_distance_m;
    m_path_start_right_distance_m = right_distance_m;

    SetupSampleRecording();
}

void PathPointsFollower::StartSegment() {
	// Get the current path segment and the current drive base distances and heading
	PathSegment& path_segment = GetPathSegment();
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);
	double gyro_heading_deg = drive_base->GetPigeonHeading();

    // Calculate the total path segment length and use it to initialise the motion profile
    double path_length = 0.0;
    int total_beziers = path_segment.m_path_definition.size();
    for (int i = 0; i < total_beziers; i++) {
        path_length += path_segment.m_path_definition[i].Length();
    }
    path_segment.m_motion_profile.InitialiseProfile(path_length);

    // Generate the path points
    m_segment_points.GeneratePathPoints(path_segment.m_path_definition, path_segment.m_motion_profile,
                                        m_follower_parameters.m_wheelbase_width_m, path_segment.m_reverse,
                                        m_follower_parameters.m_period_s);

	// Record the initial distances. Following the segment is all relative to this.
    m_segment_start_left_distance_m = left_distance_m;
    m_segment_start_right_distance_m = right_distance_m;

	// Calculate an offset from the gyro heading to the heading at the start of the path. This offset
	// will be added to all gyro values to get the heading. This means that the initial angle of
	// the robot is assumed to be correct.
	m_gyro_heading_offset_deg = m_segment_points.GetLeftPathPoints()[0].m_heading_deg - gyro_heading_deg;

	// Initialise the counter for mechanism actions
    m_mechanism_actions_done_count = 0;
	m_period_counter = 0;

	// Convert the mechanism actions time specification into a period
	double TIME_PERIOD = 0.02;
	int total_time_periods = m_segment_points.GetLeftPathPoints().size();
    for (MechanismAction& mechanism_action : path_segment.m_mechanism_actions) {
		switch (mechanism_action.m_time_specification) {
			case MechanismAction::TimeSpecification::Start:
				mechanism_action.m_time_period = mechanism_action.m_time / TIME_PERIOD;
				break;
			case MechanismAction::TimeSpecification::End:
				mechanism_action.m_time_period = total_time_periods + mechanism_action.m_time / TIME_PERIOD;
				break;
		}

        // If the action is meant to occur before the start of the segment clip it to 0 as otherwise
        // it will never be performed.
        if (mechanism_action.m_time_period < 0) {
            std::cout << "ERROR: Mechanism action " << mechanism_action.m_action << " occurrs before the start of the segment\n";
            mechanism_action.m_time_period = 0;
        }
	}

	// Clear the follower state
	m_left_follower_state.m_last_error = 0;
	m_left_follower_state.m_total_error = 0;
	m_right_follower_state.m_last_error = 0;
	m_right_follower_state.m_total_error = 0;
}

void PathPointsFollower::FollowSegment() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);
	double gyro_heading_deg = drive_base->GetPigeonHeading();

	// Calculate the motor outputs for the left and right sides
    bool finished = m_period_counter >= m_segment_points.GetTotalPoints();
    int point_index = finished ? m_segment_points.GetTotalPoints() - 1 : m_period_counter;
    const RobotPathPoints::PathPoint& left_point = m_segment_points.GetLeftPathPoints()[point_index];
    const RobotPathPoints::PathPoint& right_point = m_segment_points.GetRightPathPoints()[point_index];
	double left_feed_forward;
	double right_feed_forward;
    double left_output = FollowSide(m_left_follower_state, left_point, left_distance_m - m_segment_start_left_distance_m, left_feed_forward);
    double right_output = FollowSide(m_right_follower_state, right_point, right_distance_m - m_segment_start_right_distance_m, right_feed_forward);

	// Get the desired heading in degrees. Note that the heading for the left and right trajectories
	// will be the same (this is always true for tank drive).
	// Pathfinder heading turns in the opposite direction to the pigeon angle
	// so reverse the sign
	double desired_heading_deg = left_point.m_heading_deg;

	// Normalise the desired heading to the range [0, 360) as not all paths are defined the same
	while (desired_heading_deg >= 360.0) desired_heading_deg -= 360.0;
	while (desired_heading_deg <    0.0) desired_heading_deg += 360.0;

	// Adjust the gyro heading to get a heading relative to the path
	gyro_heading_deg += m_gyro_heading_offset_deg;

	// Normalise the gyro heading to the range [0, 360) to match the range used for paths
	while (gyro_heading_deg >= 360.0) gyro_heading_deg -= 360.0;
	while (gyro_heading_deg <    0.0) gyro_heading_deg += 360.0;

	// Calculate the different between the desired and actual heading
	// Make sure to bound this from -180 to 180, otherwise you will get super large values
	double angle_difference_deg = desired_heading_deg - gyro_heading_deg;
	while (angle_difference_deg > 180.0)  angle_difference_deg -= 360.0;
	while (angle_difference_deg < -180.0) angle_difference_deg += 360.0;

	// Calculate a turn value from the angle error. NOTE: this uses the same gain as
	// DriveBase::CalculateDriveStraightAdjustment() so it should be good.
	double turn = 0.01 * angle_difference_deg;

	// Adjust the left and right output by the turning values. Note this calculation is
	// slightly different to DriveBase::ArcadeDrive() so it should be checked. However for
	// small errors it should be fine.
	left_output = (left_output - turn);
	right_output = (right_output + turn);

//	std::cout << "(" << l << ", " << r << ") " <<
//			desired_heading_deg << " - " << gyro_heading_deg << " diff " << angle_difference_deg <<
//			"  Encoders (" << left_encoder << ", " << right_encoder << ")" <<
//			"  Errors (" << m_left_follower.last_error << ", " << m_right_follower.last_error << ")" <<
//			"\n";


	// Bound the output to the legal range of [-1, 1]. Do this by scaling both left and right
	// equally so that we don't get crazy extra turning.
	double max_speed = std::max(fabs(left_output), fabs(right_output));
	if (max_speed > 1.0) {
		left_output /= max_speed;
		right_output /= max_speed;
	}

	// Drive the robot at the required speed
	drive_base->TankDriveOpenLoop(left_output, right_output);

    // If there are still mechanism actions remaining, check if it is the right period to perform the next ones.
    // Note that multiple actions may fire in the same period.
	const std::string* mechanism_action_name = NULL;
	PathSegment& path_segment = GetPathSegment();
    for (MechanismAction& mechanism_action : path_segment.m_mechanism_actions) {
        if (mechanism_action.m_time_period == m_period_counter) {
            // Perform the mechanism action and advance to the next one
            if (GetMechanismController()) {
                GetMechanismController()->DoAction(mechanism_action.m_action);
            }
            mechanism_action_name = &(mechanism_action.m_action);
            m_mechanism_actions_done_count++;
        }
    }
	m_period_counter++;

    // Record a sample for this time period for testing purposes
	double relative_left_distance_m = left_distance_m - m_path_start_left_distance_m;
	double relative_right_distance_m = right_distance_m - m_path_start_right_distance_m;
	RecordSample(relative_left_distance_m, relative_right_distance_m, gyro_heading_deg,
				 left_output, right_output, left_feed_forward, right_feed_forward,
				 mechanism_action_name, left_point, right_point);
}

bool PathPointsFollower::IsSegmentFinished() {
    // The segment is finish when the path points have all been done and all the mechanism actions have been done
	PathSegment& path_segment = GetPathSegment();
    return m_period_counter >= m_segment_points.GetTotalPoints() &&
           m_mechanism_actions_done_count >= (int)path_segment.m_mechanism_actions.size();
}

void PathPointsFollower::FinishSegment() {
}

void PathPointsFollower::FinishPath() {
    WriteTestSampleToFile();
}


//==========================================================================
// Following Calculations

double PathPointsFollower::FollowSide(FollowerState& follower_state, const RobotPathPoints::PathPoint& point, double distance_m, double& feed_forward) {
	// Calculate the error and accumulated total error
	double error = point.m_distance - distance_m;
	double total_error = follower_state.m_total_error + error;

	// Calculate the feed forward term. It contains 3 parts.
	//  1. A term proportional to the desired velocity
	//  2. A fixed term in the direction of the desired velocity
	//  3. A term proportional to the desired acceleration
	feed_forward = m_follower_parameters.m_kv * point.m_velocity +
                   m_follower_parameters.m_ka * point.m_acceleration;
    if (point.m_velocity != 0.0) {
        if (point.m_velocity > 0.0) {
            feed_forward += m_follower_parameters.m_kv_offset;
        } else {
            feed_forward -= m_follower_parameters.m_kv_offset;
        }
    }        

	// Calculate the PID term. It contins 3 parts.
	//	1. P term 'proportional' is based on the current error
	//	2. I term 'integral' is based on the accumulated total error
	//	3. D term 'derivative' is based on the change in the errorr
	double pid_value = m_follower_parameters.m_kp * error +
						      m_follower_parameters.m_ki * total_error * m_follower_parameters.m_period_s +
							  m_follower_parameters.m_kd * ((error - follower_state.m_last_error) / m_follower_parameters.m_period_s);

	// Update the errors in the follower state for 
	follower_state.m_last_error = error;
	follower_state.m_total_error = total_error;

	// The require output is the sum of the feed forward and PID values
	return feed_forward + pid_value;
}


//==========================================================================
// Sample Recording

void PathPointsFollower::SetupSampleRecording() {
	// Clear the list of samples
	m_sample_list.clear();

	// Reset and start the timer
	m_timer.Reset();
	m_timer.Start();
}

void PathPointsFollower::RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
					double left_output, double right_output, double left_feed_forward, double right_feed_forward, const std::string* mechanism_action,
					const RobotPathPoints::PathPoint& left_point, const RobotPathPoints::PathPoint& right_point) {
    // Do nothing if we are not recording samples
    if (!GetRecordSamples()) return;

	// Record the data sample for this time step
	Sample sample;
	sample.m_time_s = m_timer.Get();
	sample.m_left_distance_m = left_distance_m;
	sample.m_right_distance_m = right_distance_m;
	sample.m_gyro_heading_deg = gyro_heading_deg;
	sample.m_left_output = left_output;
	sample.m_right_output = right_output;
	sample.m_left_feed_forward = left_feed_forward;
	sample.m_right_feed_forward = right_feed_forward;
	sample.m_mechanism_action = mechanism_action;
	sample.m_segment_index = GetPathSegmentIndex();
	sample.m_left_point = left_point;
	sample.m_right_point = right_point;
	m_sample_list.push_back(sample);			
}

void PathPointsFollower::WriteTestSampleToFile() {
    // Do nothing if we are not recording samples
    if (!GetRecordSamples()) return;

    
	const char* const RESULT_FILENAME = "/home/lvuser/TestPathFollower.csv";
	std::ofstream results_file;
	results_file.open(RESULT_FILENAME, std::ios::out | std::ios::app);
	if (results_file.fail()) {
		std::cout << "PathController::WriteTestSampleToFile() - Failed to open result file\n";
		return;
	}

	// Write the parameters used for the test
	results_file << "\"Path\",\"" << GetRobotPath()->m_name << "\"\n";
	results_file << "\"Follower\",\"PathPointsFollower\"\n";
	results_file << "\"kv\",\"" << m_follower_parameters.m_kv << "\"\n";
	results_file << "\"kv offset\",\"" << m_follower_parameters.m_kv_offset << "\"\n";
	results_file << "\"ka\",\"" << m_follower_parameters.m_ka << "\"\n";
	results_file << "\"kp\",\"" << m_follower_parameters.m_kp << "\"\n";
	results_file << "\"ki\",\"" << m_follower_parameters.m_ki << "\"\n";
	results_file << "\"kd\",\"" << m_follower_parameters.m_kd << "\"\n";
	results_file << "\"Wheelbase Width (m)\",\"" << m_follower_parameters.m_wheelbase_width_m << "\"\n";
	results_file << "\"Time Period (s)\",\"" << m_follower_parameters.m_period_s << "\"\n";

    // Write the recorded sample data for the test. Each sample parameter is on a separate line
	int total_samples = m_sample_list.size();

	// Write the desired path heading (same for left and right)
	results_file << "\"Heading\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(2) << m_sample_list[i].m_left_point.m_heading_deg;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the left side desired path
	results_file << "\"XLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_point.m_position.x;
	results_file << "\n";
	results_file << "\"YLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_point.m_position.y;
	results_file << "\n";
	results_file << "\"PositionLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_point.m_distance;
	results_file << "\n";
	results_file << "\"VelocityLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_point.m_velocity;
	results_file << "\n";
	results_file << "\"AccelerationLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_point.m_acceleration;
	results_file << "\n";

	// Write the right side desired path
	results_file << "\"XRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_point.m_position.x;
	results_file << "\n";
	results_file << "\"YRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_point.m_position.y;
	results_file << "\n";
	results_file << "\"PositionRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_point.m_distance;
	results_file << "\n";
	results_file << "\"VelocityRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_point.m_velocity;
	results_file << "\n";
	results_file << "\"AccelerationRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_point.m_acceleration;
	results_file << "\n";

	// Write the sample times in a single line
	results_file << "\"Time\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_time_s;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the left and right encoder positions
	results_file << "\"Left Distance\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_left_distance_m;
	results_file << "\n";
	results_file << "\"Right Distance\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_right_distance_m;
	results_file << "\n";

	// Write the recorded gyro heading
	results_file << "\"Gyro Heading\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(2) << m_sample_list[i].m_gyro_heading_deg;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the left and right motor drive outputs and feed forwards
	results_file << "\"Left Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_output;
	results_file << "\n";
	results_file << "\"Right Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_output;
	results_file << "\n";
	results_file << "\"Left Feedforward\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_feed_forward;
	results_file << "\n";
	results_file << "\"Right Feedforward\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_feed_forward;
	results_file << "\n";

	// Write the segment index
	results_file << "\"Segment Index\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_segment_index;
	results_file << "\n";

	// Write the mechanism action, if any
	results_file << "\"Mechanism Action\"";
	for (int i = 0; i < total_samples; i++) {
		results_file << ",";
		if (m_sample_list[i].m_mechanism_action != NULL) results_file << *(m_sample_list[i].m_mechanism_action);
	}
	results_file << "\n";

	// Write a completely blank line to mark the end of this test
	results_file << "\n";

	// Log some metrics about the overall error for testing purposes
	double left_error_total = 0.0;
	double right_error_total = 0.0;
	for (int i = 0; i < total_samples; i++) {
		const Sample sample = m_sample_list[i];
		left_error_total = abs(sample.m_left_distance_m - sample.m_left_point.m_distance);
		right_error_total = abs(sample.m_right_distance_m - sample.m_right_point.m_distance);
	}
	std::cout << "Left Error Mean " << (left_error_total/total_samples) << "\n";
	std::cout << "Right Error Mean " << (right_error_total/total_samples) << "\n";
}
