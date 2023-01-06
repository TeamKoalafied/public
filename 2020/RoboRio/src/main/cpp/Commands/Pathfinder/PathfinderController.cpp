//==============================================================================
// PathfinderController.cpp
//==============================================================================

#include "PathfinderController.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>


//double my_pathfinder_follow_encoder2(EncoderConfig c, EncoderFollower *follower, Segment s, int trajectory_length, int encoder_tick) {
//    double distance_covered = ((double)encoder_tick - (double)c.initial_position) /  ((double)c.ticks_per_revolution);
//    distance_covered = distance_covered * c.wheel_circumference;
//
//    if (follower->segment < trajectory_length) {
//        follower->finished = 0;
//        double error = s.position - distance_covered;
//        double calculated_value = c.kp * error +
//                                  c.kd * ((error - follower->last_error) / s.dt) +
//                                  (c.kv * s.velocity + c.ka * s.acceleration);
//
////    	std::cout << "distance_covered" << distance_covered << " s.position " << s.position << "\n";
//
//
//        follower->last_error = error;
//        follower->heading = s.heading;
//        follower->output = calculated_value;
//        follower->segment = follower->segment + 1;
//        return calculated_value;
//    } else {
//        follower->finished = 1;
//        return 0.0;
//    }
//}
//
//double my_pathfinder_follow_encoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
//    int segment = follower->segment;
//    if (segment >= trajectory_length) {
//        follower->finished = 1;
//        follower->output = 0.0;
//        Segment last = trajectory[trajectory_length - 1];
//        follower->heading = last.heading;
//        return 0.0;
//    } else {
//        return my_pathfinder_follow_encoder2(c, follower, trajectory[segment], trajectory_length, encoder_tick);
//    }
//}


//==============================================================================
// Construction and Destruction

PathfinderController::PathfinderController(Segment* left_trajectory, Segment * right_trajectory,
										   int trajectory_length, const char* name, bool own_trajectory) :
	PathController() {

//    int initial_position, ticks_per_revolution;
//    double wheel_circumference;
//    double kp, ki, kd, kv, ka;

	SetEncoderCountPerRevolution(4096);
	SetWheelCircumferenceM(M_PI * 6.25 * 0.0254);
	SetPID(1.0, 0.0, 0.0);

	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
	m_left_encoder_config.kv = 1.0/4.28;
	m_right_encoder_config.kv = 1.0/4.28;

	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V
	m_left_encoder_config.ka = 1.0/18.29;
	m_right_encoder_config.ka = 1.0/18.29;
	m_left_encoder_config.initial_position = 0;
	m_right_encoder_config.initial_position = 0;

	InitialiseTrajectoryFromArrays(left_trajectory, right_trajectory, trajectory_length, name, own_trajectory);

	m_mechanism_actions = NULL;
    m_mechanism_actions_count = 0;
}

PathfinderController::~PathfinderController() {
	//DeleteTrajectory();
	if (m_own_trajectory) {
		delete[] m_left_trajectory;
		delete[] m_right_trajectory;
	}
}

//==============================================================================
// Robot Setup

void PathfinderController::SetEncoderConfig(const EncoderConfig encoder_config) {
	m_left_encoder_config = encoder_config;
	m_right_encoder_config = encoder_config;
}

void PathfinderController::SetEncoderCountPerRevolution(int encoder_count_per_revolution) {
	m_left_encoder_config.ticks_per_revolution = encoder_count_per_revolution;
	m_right_encoder_config.ticks_per_revolution = encoder_count_per_revolution;
}

void PathfinderController::SetWheelCircumferenceM(double wheel_circumference_m) {
	m_left_encoder_config.wheel_circumference = wheel_circumference_m;
	m_right_encoder_config.wheel_circumference = wheel_circumference_m;
}

void PathfinderController::SetPID(double p, double i, double d) {
	m_left_encoder_config.kp = p;
	m_right_encoder_config.kp = p;
	m_left_encoder_config.ki = i;
	m_right_encoder_config.ki = i;
	m_left_encoder_config.kd = d;
	m_right_encoder_config.kd = d;
}


//==============================================================================
// Trajectory Setup

void PathfinderController::InitialiseTrajectoryFromArrays(Segment* left_trajectory, Segment * right_trajectory,
														  int trajectory_length, const char* name, bool own_trajectory) {
	m_left_trajectory = left_trajectory;
	m_right_trajectory = right_trajectory;
	m_trajectory_length = trajectory_length;
	m_trajectory_name = name;
	m_own_trajectory = own_trajectory;

	m_left_follower.last_error = 0.0;
	m_left_follower.heading = 0.0;
	m_left_follower.output = 0.0;
	m_left_follower.segment = 0;
	m_left_follower.finished = 0;

	m_right_follower.last_error = 0.0;
	m_right_follower.heading = 0.0;
	m_right_follower.output = 0.0;
	m_right_follower.segment = 0;
	m_right_follower.finished = 0;
}

//==============================================================================
// Mechanism Actions Setup

void PathfinderController::SetMechanismActions(IMechanismController* mechanism_controller,
											   MechanismAction* mechanism_actions,
											   int mechanism_actions_count) {
	m_mechanism_controller = mechanism_controller;
	m_mechanism_actions = mechanism_actions;
	m_mechanism_actions_count = mechanism_actions_count;
}

//==============================================================================
// Trajectory Following (from PathController)

void PathfinderController::StartTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg) {
	// Record the initial encoder data. Following the path is all relative to this.
	m_left_encoder_config.initial_position = left_encoder;
	m_right_encoder_config.initial_position = right_encoder;

	// Calculate an offset from the gyro heading to the heading at the start of the path. This offset
	// will be added to all gyro values to get the heading. This means that the initial angle of
	// the robot is assumed to be correct.
	m_gyro_heading_offset_deg = m_left_trajectory[0].heading * 180.0 / M_PI - gyro_heading_deg;

	// Set up sample recording
	SetupSampleRecording();

	// Initialise the counter for mechanism actions
	m_mechanism_actions_index = 0;
	m_period_counter = 0;

	// Convert the mechanism actions time specification into a period
	double TIME_PERIOD = 0.02;
	int total_time_periods = m_trajectory_length;
	for (int i = 0; i < m_mechanism_actions_count; i++) {
		MechanismAction& mechanism_action = m_mechanism_actions[i];
		switch (mechanism_action.m_time_specification) {
			case MechanismAction::TimeSpecification::Start:
				mechanism_action.m_time_period = mechanism_action.m_time / TIME_PERIOD;
				break;
			case MechanismAction::TimeSpecification::End:
				mechanism_action.m_time_period = total_time_periods + mechanism_action.m_time / TIME_PERIOD;
				break;
		}
	}
}

const std::string* PathfinderController::FollowTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg,
 														  double& left_output, double& right_output) {
	// Calculate the motor outputs for the left and right trajectories
	double l = FollowEncoder(m_left_encoder_config, &m_left_follower, m_left_trajectory,
									     m_trajectory_length, left_encoder);
	double r = FollowEncoder(m_right_encoder_config, &m_right_follower, m_right_trajectory,
									     m_trajectory_length, right_encoder);

	// Get the desired heading in degrees. Note that the heading for the left and right trajectories
	// will be the same (this is always true for tank drive).
	// Pathfinder heading turns in the opposite direction to the pigeon angle
	// so reverse the sign
	double desired_heading_deg = m_left_follower.heading * 180.0 / M_PI;

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
	left_output = (l - turn);
	right_output = (r + turn);

//	std::cout << "(" << l << ", " << r << ") " <<
//			desired_heading_deg << " - " << gyro_heading_deg << " diff " << angle_difference_deg <<
//			"  Encoders (" << left_encoder << ", " << right_encoder << ")" <<
//			"  Errors (" << m_left_follower.last_error << ", " << m_right_follower.last_error << ")" <<
//			"\n";

	double left_distance_m = (left_encoder - m_left_encoder_config.initial_position)*
			m_left_encoder_config.wheel_circumference/m_left_encoder_config.ticks_per_revolution;
	double right_distance_m = (right_encoder - m_right_encoder_config.initial_position)*
			m_right_encoder_config.wheel_circumference/m_right_encoder_config.ticks_per_revolution;

	// Bound the output to the legal range of [-1, 1]. Do this by scaling both left and right
	// equally so that we don't get crazy extra turning.
	double max_speed = std::max(fabs(left_output), fabs(right_output));
	if (max_speed > 1.0) {
		left_output /= max_speed;
		right_output /= max_speed;
	}

	// Do mechanism actions if there are any
	const std::string* mechanism_action = NULL;
	if (m_mechanism_actions != NULL) {
		// If there are still mechanism actions remaining, check if it is the right period to perform the next ones.
		// Note that multiple actions may fire in the same period.
		while (m_mechanism_actions_index < m_mechanism_actions_count &&
		       m_mechanism_actions[m_mechanism_actions_index].m_time_period == m_period_counter) {
			// Perform the mechanism action and advance to the next one
			if (m_mechanism_controller) {
				m_mechanism_controller->DoAction(m_mechanism_actions[m_mechanism_actions_index].m_action);
			}
			mechanism_action = &m_mechanism_actions[m_mechanism_actions_index].m_action;
			m_mechanism_actions_index++;
		}
	}
	m_period_counter++;

	// Record the data sample for this time step
	RecordSample(left_distance_m, right_distance_m, gyro_heading_deg, left_output, right_output, mechanism_action);

	return mechanism_action;
}

bool PathfinderController::IsTrajectoryFinished() {
	// Regard the trajectory as finished if either the left or right has finished.
	// They should both finish at the same time, but test both to be safe.
	return (m_left_follower.finished || m_right_follower.finished) && m_mechanism_actions_index == m_mechanism_actions_count;
}

Segment* PathfinderController::GetLeftSegment() const
{
    if (m_left_follower.segment >= m_trajectory_length) {
		return &m_left_trajectory[m_trajectory_length - 1];
    }
	else {
		return &m_left_trajectory[m_left_follower.segment];
	}
}

Segment* PathfinderController::GetRightSegment() const
{
    if (m_right_follower.segment >= m_trajectory_length) {
		return &m_right_trajectory[m_trajectory_length - 1];
    }
	else {
		return &m_right_trajectory[m_right_follower.segment];
	}
}


//==========================================================================
// Sample Recording(from PathController)

void PathfinderController::WriteSampleHeading(std::ofstream& results_file)
{
	results_file << "\"Path\",\"" << m_trajectory_name << "\"\n";
	results_file << "\"Follower\",\"" << FollowerName() << "\"\n";
	results_file << "\"kv\",\"" << m_left_encoder_config.kv << "\"\n";
	results_file << "\"ka\",\"" << m_left_encoder_config.ka << "\"\n";
	results_file << "\"p\",\"" << m_left_encoder_config.kp << "\"\n";
	results_file << "\"i\",\"" << m_left_encoder_config.ki << "\"\n";
	results_file << "\"d\",\"" << m_left_encoder_config.kd << "\"\n";

	// Write the heading
	results_file << "\"Heading\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].heading;
	results_file << "\n";

	results_file << "\"XLeft\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].x;
	results_file << "\n";
	results_file << "\"YLeft\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].y;
	results_file << "\n";
	results_file << "\"PositionLeft\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].position;
	results_file << "\n";
	results_file << "\"VelocityLeft\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].velocity;
	results_file << "\n";
	results_file << "\"AccelerationLeft\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_left_trajectory[i].acceleration;
	results_file << "\n";

	results_file << "\"XRight\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_right_trajectory[i].x;
	results_file << "\n";
	results_file << "\"YRight\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_right_trajectory[i].y;
	results_file << "\n";
	results_file << "\"PositionRight\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_right_trajectory[i].position;
	results_file << "\n";
	results_file << "\"VelocityRight\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_right_trajectory[i].velocity;
	results_file << "\n";
	results_file << "\"AccelerationRight\"";
	for (int i = 0; i < m_trajectory_length; i++) results_file << "," << std::setprecision(3) << m_right_trajectory[i].acceleration;
	results_file << "\n";

	// results_file << "\"Path\",\"" << m_trajectory_name << "\"\n";
	// results_file << "\"Follower\",\"" << FollowerName() << "\"";
	// results_file << ",\"kv\",\"" << m_left_encoder_config.kv << "\"";
	// results_file << ",\"ka\",\"" << m_left_encoder_config.ka << "\"";
	// results_file << ",\"p\",\"" << m_left_encoder_config.kp << "\"";
	// results_file << ",\"i\",\"" << m_left_encoder_config.ki << "\"";
	// results_file << ",\"d\",\"" << m_left_encoder_config.kd << "\"";
	// results_file << "\n";
}
