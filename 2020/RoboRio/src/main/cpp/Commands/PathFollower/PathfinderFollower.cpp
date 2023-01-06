//==============================================================================
// PathfinderFollower.cpp
//==============================================================================

#include "PathfinderFollower.h"

#include "RobotPathPoints.h"
#include "../RobotPath/IPathDriveBase.h"
#include "../RobotPath/PathSegment.h"
#include "../RobotPath/RobotPath.h"
#include "../Pathfinder/JaciPathfinderController.h"

#include <fstream>
#include <iostream>
#include <iomanip>


//==========================================================================
// Construction and Destruction

PathfinderFollower::PathfinderFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
					   bool record_samples, const EncoderConfig& encoder_config) :
	PathFollower(robot_path, drive_base, mechanism_controller, record_samples) {

    m_encoder_config = encoder_config;

    m_path_controller = NULL;
}

PathfinderFollower::~PathfinderFollower() {
}


//==========================================================================
// Path Following (from IPathFollower)

void PathfinderFollower::StartPath() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);
	// double gyro_heading_deg = drive_base->GetPigeonHeading();

	// Record the start distances for the whole path
    m_path_start_left_distance_m = left_distance_m;
    m_path_start_right_distance_m = right_distance_m;

    SetupSampleRecording();
}

void PathfinderFollower::StartSegment() {
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
    //std::cout << path_segment.m_name << " time " << (path_segment.m_motion_profile.GetTotalTime() /0.02) << " periods\n";

    // Generate the path points
    // NMBTODO Wheelbase and time period should be passed in as parameters (but do we care?)
    double wheel_base = 0.72;
    double time_period_s = 0.02; // 20ms
    RobotPathPoints robot_path;
    robot_path.GeneratePathPoints(path_segment.m_path_definition, path_segment.m_motion_profile,
                                  wheel_base, path_segment.m_reverse, time_period_s);

    // Convert the path points we have calculated into the Segments used by the Pathfinder code
    int trajectory_length = robot_path.GetLeftPathPoints().size();
    Segment* left_trajectory = new Segment[trajectory_length];
    Segment* right_trajectory = new Segment[trajectory_length];
    double dt = robot_path.GetLeftPathPoints()[1].m_time - robot_path.GetLeftPathPoints()[0].m_time;

    for (int i = 0; i < trajectory_length; i++) {
        Segment& left_segment = left_trajectory[i];
        const RobotPathPoints::PathPoint& left_point = robot_path.GetLeftPathPoints()[i];
        left_segment.dt = dt;
        left_segment.x = left_point.m_position.x;
        left_segment.y = left_point.m_position.y;
        left_segment.position = left_point.m_distance;
        left_segment.velocity = left_point.m_velocity;
        left_segment.acceleration = left_point.m_acceleration;
        left_segment.jerk = left_point.m_jerk;
        left_segment.heading = left_point.m_heading_deg * M_PI / 180.0;
        Segment& right_segment = right_trajectory[i];
        const RobotPathPoints::PathPoint& right_point = robot_path.GetRightPathPoints()[i];
        right_segment.dt = dt;
        right_segment.x = right_point.m_position.x;
        right_segment.y = right_point.m_position.y;
        right_segment.position = right_point.m_distance;
        right_segment.velocity = right_point.m_velocity;
        right_segment.acceleration = right_point.m_acceleration;
        right_segment.jerk = right_point.m_jerk;
        right_segment.heading = right_point.m_heading_deg * M_PI / 180.0;
    }

    // Create the controller and set up its configuation and mechanism actions. Also
    // set to not record samples as we are doing that.
    PathfinderController* path_controller = new JaciPathfinderController(left_trajectory, right_trajectory,
                                                                        trajectory_length, path_segment.m_name.c_str(), true);
    path_controller->SetEncoderConfig(m_encoder_config);
    path_controller->SetRecordSamples(false);
    path_controller->SetMechanismActions(GetMechanismController(), &path_segment.m_mechanism_actions[0], path_segment.m_mechanism_actions.size());
    m_path_controller = path_controller;

    // Calculate the encoder position from the distance
   	// double encoder_to_distance = M_PI * 6.25 * 0.0254 / 4096.0;
   	double distance_to_encoder = 4096.0/ (M_PI * 6.25 * 0.0254);
    int left_encoder = left_distance_m * distance_to_encoder;
    int right_encoder = right_distance_m * distance_to_encoder;

    // Start the Pathfinder trajectory
	m_path_controller->StartTrajectory(left_encoder, right_encoder, gyro_heading_deg);
}

void PathfinderFollower::FollowSegment() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);
	double gyro_heading_deg = drive_base->GetPigeonHeading();

    // Record the current segment before doing FollowTrajectory() as it will advance to the next one
	Segment* left_segment = m_path_controller->GetLeftSegment();
    Segment* right_segment = m_path_controller->GetRightSegment();

    // Calculate the encoder position from the distance
   	// double encoder_to_distance = M_PI * 6.25 * 0.0254 / 4096.0;
   	double distance_to_encoder = 4096.0/ (M_PI * 6.25 * 0.0254);
    int left_encoder = left_distance_m * distance_to_encoder;
    int right_encoder = right_distance_m * distance_to_encoder;

    // Get Pathfinder to follow the trajectory
	double left_output;
	double right_output;
	const std::string* mechanism_action = m_path_controller->FollowTrajectory(left_encoder, right_encoder,
																			  gyro_heading_deg, left_output, right_output);

	// Drive the robot at the required speed
	drive_base->TankDriveOpenLoop(left_output, right_output);

    // Record a sample for this time period for testing purposes
	double relative_left_distance_m = left_distance_m - m_path_start_left_distance_m;
	double relative_right_distance_m = right_distance_m - m_path_start_right_distance_m;
    RecordSample(relative_left_distance_m, relative_right_distance_m, gyro_heading_deg, left_output, right_output,
                 mechanism_action, left_segment,  right_segment);
}

bool PathfinderFollower::IsSegmentFinished() {
	return m_path_controller->IsTrajectoryFinished();
}

void PathfinderFollower::FinishSegment() {
}

void PathfinderFollower::FinishPath() {
    WriteTestSampleToFile();
}


//==========================================================================
// Sample Recording

void PathfinderFollower::SetupSampleRecording() {
	// Clear the list of samples
	m_sample_list.clear();

	// Reset and start the timer
	m_timer.Reset();
	m_timer.Start();
}

void PathfinderFollower::RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
					double left_output, double right_output, const std::string* mechanism_action,
					Segment* left_segment, Segment* right_segment) {

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
	sample.m_mechanism_action = mechanism_action;
	sample.m_segment_index = GetPathSegmentIndex();
	if (left_segment) {
		sample.m_left_trajectory = *left_segment;
	}
	if (right_segment) {
		sample.m_right_trajectory = *right_segment;
	}
	m_sample_list.push_back(sample);
}

void PathfinderFollower::WriteTestSampleToFile() {
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
	results_file << "\"Follower\",\"PathfinderFollower\"\n";
	results_file << "\"kv\",\"" << m_encoder_config.kv << "\"\n";
	results_file << "\"ka\",\"" << m_encoder_config.ka << "\"\n";
	results_file << "\"p\",\"" << m_encoder_config.kp << "\"\n";
	results_file << "\"i\",\"" << m_encoder_config.ki << "\"\n";
	results_file << "\"d\",\"" << m_encoder_config.kd << "\"\n";

    // Write the recorded sample data for the test. Each sample parameter is on a separate line
	int total_samples = m_sample_list.size();

	// Write the desired path heading (same for left and right)
	results_file << "\"Heading\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(2) << m_sample_list[i].m_left_trajectory.heading * 180.0 / M_PI;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the left side desired path
	results_file << "\"XLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_trajectory.x;
	results_file << "\n";
	results_file << "\"YLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_trajectory.y;
	results_file << "\n";
	results_file << "\"PositionLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_trajectory.position;
	results_file << "\n";
	results_file << "\"VelocityLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_trajectory.velocity;
	results_file << "\n";
	results_file << "\"AccelerationLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_trajectory.acceleration;
	results_file << "\n";

	// Write the right side desired path
	results_file << "\"XRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_trajectory.x;
	results_file << "\n";
	results_file << "\"YRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_trajectory.y;
	results_file << "\n";
	results_file << "\"PositionRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_trajectory.position;
	results_file << "\n";
	results_file << "\"VelocityRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_trajectory.velocity;
	results_file << "\n";
	results_file << "\"AccelerationRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_trajectory.acceleration;
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

	// Write the left and right motor drive outputs
	results_file << "\"Left Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_left_output;
	results_file << "\n";
	results_file << "\"Right Output\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_right_output;
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
}

