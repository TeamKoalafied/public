//==============================================================================
// PurePursuitFollower.h
//==============================================================================

#include "PurePursuitFollower.h"

#include "../RobotPath/Geometry.h"
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

PurePursuitFollower::PurePursuitFollower(RobotPath* robot_path, IPathDriveBase* drive_base, IMechanismController* mechanism_controller,
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
    m_follower_parameters.m_path_point_spacing = 0.2;  // Spacing between generated path points
    m_follower_parameters.m_max_velocity = 0.5;
    m_follower_parameters.m_max_acceleration = 0.25;
    m_follower_parameters.m_max_velocity_curve = 2.0;
    m_follower_parameters.m_lookahead_distance = 0.5;
    m_follower_parameters.m_curvature_gain = 1.8;
}

PurePursuitFollower::~PurePursuitFollower() {
}


//==========================================================================
// Path Following (from IPathFollower)

void PurePursuitFollower::StartPath() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);

    m_path_start_left_distance_m = left_distance_m;
    m_path_start_right_distance_m = right_distance_m;

    SetupSampleRecording();
}

void PurePursuitFollower::StartSegment() {
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
	GenerateSegmentPathPoints();

	// Record the initial distances. Following the segment is all relative to this.
    m_segment_start_left_distance_m = left_distance_m;
    m_segment_start_right_distance_m = right_distance_m;

	// Calculate an offset from the gyro heading to the heading at the start of the path. This offset
	// will be added to all gyro values to get the heading. This means that the initial angle of
	// the robot is assumed to be correct.
	m_gyro_heading_offset_deg = m_path_points[0].m_heading_degrees - gyro_heading_deg;

	// Initialise the counter for mechanism actions
    m_mechanism_actions_done_count = 0;
	m_period_counter = 0;

	// Convert the mechanism actions time specification into a period
//	int total_time_periods = m_segment_points.GetLeftPathPoints().size();
    for (MechanismAction& mechanism_action : path_segment.m_mechanism_actions) {
		switch (mechanism_action.m_time_specification) {
			case MechanismAction::TimeSpecification::Start:
				mechanism_action.m_time_period = (int)(mechanism_action.m_time / m_follower_parameters.m_period_s);
				break;
			case MechanismAction::TimeSpecification::End:
//				mechanism_action.m_time_period = total_time_periods + mechanism_action.m_time / TIME_PERIOD;
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

void PurePursuitFollower::FollowSegment() {
	// Get the current drive base distances and heading
	IPathDriveBase* drive_base = GetDriveBase();
	double left_distance_m;
	double right_distance_m;
    drive_base->GetWheelDistancesM(left_distance_m, right_distance_m);
	double gyro_heading_deg = drive_base->GetPigeonHeading();

	// Record the data sample for this time step
	Sample sample;
	sample.m_time_s = m_timer.Get();
	sample.m_left_distance_m = left_distance_m;
	sample.m_right_distance_m = right_distance_m;
	sample.m_gyro_heading_deg = gyro_heading_deg;

	// Calculate the motor outputs for the left and right sides
    double left_output;
    double right_output;
    CalculatePurePursuitDrive(left_output, right_output, sample);

	// Drive the robot at the required speed
    if (m_segment_finished) {
        drive_base->Stop();
    } else {
    	drive_base->TankDriveOpenLoop(left_output, right_output);
    }

    // If there are still mechanism actions remaining, check if it is the right period to perform the next ones.
    // Note that multiple actions may fire in the same period.
	sample.m_mechanism_action = NULL;
	PathSegment& path_segment = GetPathSegment();
    for (MechanismAction& mechanism_action : path_segment.m_mechanism_actions) {
        if (mechanism_action.m_time_period == m_period_counter) {
            // Perform the mechanism action and advance to the next one
            if (GetMechanismController()) {
                GetMechanismController()->DoAction(mechanism_action.m_action);
            }
            sample.m_mechanism_action = &(mechanism_action.m_action);
            m_mechanism_actions_done_count++;
        }
    }
	m_period_counter++;

    // Record a sample for this time period for testing purposes
	double relative_left_distance_m = left_distance_m - m_path_start_left_distance_m;
	double relative_right_distance_m = right_distance_m - m_path_start_right_distance_m;
    sample.m_left_distance_m = relative_left_distance_m;
    sample.m_right_distance_m = relative_right_distance_m;
	sample.m_left_output = left_output;
	sample.m_right_output = right_output;
	sample.m_segment_index = GetPathSegmentIndex();
    if (GetRecordSamples()) m_sample_list.push_back(sample);			
}

bool PurePursuitFollower::IsSegmentFinished() {
    // The segment is finish when the flag for travelling the distance is set and
    // all the mechanism actions have been done
	PathSegment& path_segment = GetPathSegment();
    return m_segment_finished &&
           m_mechanism_actions_done_count >= (int)path_segment.m_mechanism_actions.size();
}

void PurePursuitFollower::FinishSegment() {
    // Copy all the path point from the vector for this segment to the overall vector for the
    // whole path so that they can be logged
    m_total_path_points.insert(m_total_path_points.end(), m_path_points.begin(), m_path_points.end());
}

void PurePursuitFollower::FinishPath() {
   	const char* const RESULT_FILENAME = "/home/lvuser/TestPathFollower.csv";
    WriteTestSampleToFile(RESULT_FILENAME);
}

//==========================================================================
// Testing

void PurePursuitFollower::TestGeneratePathToFile(const char* filename)
{
	GenerateSegmentPathPoints();
	WriteTestSampleToFile(filename);
}



//==========================================================================
// Pure Pursuit Calculations

void PurePursuitFollower::GenerateSegmentPathPoints()
{
	// Get the lengths of all the beziers in the current path segment
	PathSegment& path_segment = GetPathSegment();
	int total_beziers = path_segment.m_path_definition.size();
	std::vector<double> segment_lengths(total_beziers);
	double total_length = 0;
	for (int i = 0; i < total_beziers; i++) {
		segment_lengths[i] = path_segment.m_path_definition[i].Length();
		total_length += segment_lengths[i];
	}

    // Calculate the total number of points for the required spacing and create space
    // for them, plus the 'extension' point.
    double point_spacing = m_follower_parameters.m_path_point_spacing;
	int total_points = (int)ceil(total_length / point_spacing) + 1;
	m_path_points.resize(total_points + 1);

	// Loop through the array setting up the points
	int bezier_index = 0;
	double total_pos = 0.0;
	double bezier_pos = 0.0;
	for (int i = 0; i < total_points; i++) {
		PathPoint& path_point = m_path_points[i];

        // Record the distance along the path
        path_point.m_distance = total_pos;

        // Calculate the position by measuring along the arc of the current bezier segment
        Bezier3& bezier = path_segment.m_path_definition[bezier_index];
        double u;
        path_point.m_position = bezier.ArcLengthPoint(bezier_pos, u);

        // Get the derivative of the bezier and convert it to a heading in degrees, normalised to [0, 360)
        Point2D direction = bezier.Derivative(u);
        double angle_rad = atan2(direction.y, direction.x);
        double heading_degrees = angle_rad * 180.0 / M_PI;
        while (heading_degrees >= 360.0) heading_degrees -= 360.0;
        while (heading_degrees <    0.0) heading_degrees += 360.0;
        path_point.m_heading_degrees = heading_degrees;

        // Get the curvature and record the maximum velocity. The maximum velocity is reduced if the
        // curvature is too large. Note that curvature is signed (+ve left)
        path_point.m_curvature = bezier.Curvature(u);
        path_point.m_max_velocity = m_follower_parameters.m_max_velocity;
        if (path_point.m_curvature != 0.0) {
            // NMBTODO Consider determining the max velocity from a maximum centripetal acceleration
            // v = sqrt(a/C)
            double curve_max_velocity = m_follower_parameters.m_max_velocity_curve / fabs(path_point.m_curvature);
            if (path_point.m_max_velocity > curve_max_velocity) {
                path_point.m_max_velocity = curve_max_velocity;
            }
        }

        // Update the total position and the position relative to the current bezier segment.
        total_pos += point_spacing;
        bezier_pos += point_spacing;

        // If we have advanaced past the end of the current bezier segment move on to the next one, but
        // don't go past the last one, just move to its exact length instead.
        if (bezier_pos > segment_lengths[bezier_index]) {
            bezier_pos -= segment_lengths[bezier_index];
            if (bezier_index < total_beziers - 1) {
                bezier_index++;
            }
            else {
                bezier_pos = segment_lengths[bezier_index];
            }
        }
	}

    // Start the path with zero velocity and move forward along it accelerating up to the maximum
    // allowed velocity for each point.
    double velocity = 0.0;
    m_path_points[0].m_velocity = 0.0;
 	for (int i = 1; i < total_points; i++) {
        double distance = m_path_points[i].m_distance - m_path_points[i - 1].m_distance;
        velocity = sqrt(2*distance * m_follower_parameters.m_max_acceleration + velocity*velocity);
        if (velocity > m_path_points[i].m_max_velocity) {
            velocity = m_path_points[i].m_max_velocity;
        }
        m_path_points[i].m_velocity = velocity;
    }

    // End the path with zero velocity and move backwards along it accelerating up to the current
    // velocity from the foward sweep for each point.
    velocity = 0.0;
    m_path_points[total_points - 1].m_velocity = 0.0;
 	for (int i = total_points - 2; i >= 0; i--) {
        double distance = m_path_points[i + 1].m_distance - m_path_points[i].m_distance;
        velocity = sqrt(2*distance * m_follower_parameters.m_max_acceleration + velocity*velocity);
        if (velocity > m_path_points[i].m_velocity) {
            velocity = m_path_points[i].m_velocity;
        }
        m_path_points[i].m_velocity = velocity;
    }

    // Calculate the acceleration for each point
 	for (int i = 0; i < total_points - 1; i++) {
        double velocity = m_path_points[i].m_velocity;
        double final_velocity = m_path_points[i + 1].m_velocity;
        double distance = m_path_points[i + 1].m_distance - m_path_points[i].m_distance;
        m_path_points[i].m_acceleration = (final_velocity * final_velocity - velocity * velocity)/(2*distance);
    }

    // Add an extra 'extension' point to the end of the points list. This gives a long straight segment
    // where we can look for the 'lookahead' point as we come to the end of the path.
	double extension_distance = 1.0;
	double extension_heading_radians = m_path_points[total_points - 1].m_heading_degrees * M_PI / 180.0;
	Point2D extension_position = m_path_points[total_points - 1].m_position;
	extension_position.x += extension_distance * cos(extension_heading_radians);
	extension_position.y += extension_distance * sin(extension_heading_radians);
	m_path_points[total_points].m_distance = m_path_points[total_points - 1].m_distance + extension_distance;
	m_path_points[total_points].m_position = extension_position;
	m_path_points[total_points].m_velocity = 0.0;
	m_path_points[total_points].m_acceleration = 0.0;
	m_path_points[total_points].m_curvature = 0.0;
	m_path_points[total_points].m_heading_degrees = m_path_points[total_points - 1].m_heading_degrees;
	m_path_points[total_points].m_max_velocity = 0.0;

    // If the path is reverse then flip everything around
    if (path_segment.m_reverse) {
 	    for (int i = 0; i <= total_points; i++) {
            PathPoint& path_point = m_path_points[i];
			double heading = path_point.m_heading_degrees + 180;
			if (heading > 360.0) heading -= 360.0;
			path_point.m_heading_degrees = heading;

            path_point.m_distance     = -path_point.m_distance;
            path_point.m_velocity     = -path_point.m_velocity;
            path_point.m_acceleration = -path_point.m_acceleration;
            path_point.m_curvature    = -path_point.m_curvature;

            // Note: the position and max velocity stay the same
        }
    }

    m_closest_index = 0;
    m_lookahead_index = 0;
    m_segment_finished = false;
}

void PurePursuitFollower::CalculatePurePursuitDrive(double& left_output, double& right_output, Sample &sample)
{
    // Get the current robot position and heading
	IPathDriveBase* drive_base = GetDriveBase();
    double position_x_m;
    double position_y_m;
    double heading_degrees;
    drive_base->GetPositionM(position_x_m, position_y_m, heading_degrees);
    Point2D robot_position(position_x_m, position_y_m);
    sample.m_robot_position = robot_position;
     
    // Get the closest point on the path to the robot position, and the desired
    // velocity and acceleration for that point.
    double path_velocity;
    double path_acceleration;
    double path_curvature;
    Point2D closest_point = GetClosestPoint(robot_position, path_velocity, path_acceleration, path_curvature);
    sample.m_closest_point = closest_point;
    sample.m_closest_index = m_closest_index;

    // Get the lookahead point. This is a point a short way ahead on the path that we will aim
    // the robot towards.
    double closest_point_distance = (closest_point - robot_position).Length();
    Point2D lookahead_point = GetLookAheadPoint(robot_position, closest_point_distance);
    sample.m_lookahead_point = lookahead_point;
    sample.m_lookahead_index = m_lookahead_index;

    // If the path is being traversed backwards we need to 'lookahead' out the back of the
    // robot and hence we need to flip the robot heading
    if (GetPathSegment().m_reverse) {
        heading_degrees += 180;
    }

    // Calculate a unit vector pointing to the left of the robot (+90 degrees from the heading)
    // By using the left the signed curvature is +ve to the left, which is our convention.
    Point2D right_vector = Point2D::UnitVectorDegrees(heading_degrees + 90);
 
    // Calculate the sideways signed distance to the lookahead point by projecting the vector
    // to the lookahead onto the right direction vector.
    Point2D lookahead_relative = lookahead_point - robot_position;
    double side_distance = lookahead_relative.Dot(right_vector);

    // Calculate the curvature to the lookahead using the pure pursuit equation. If going in reverse
    // the curvature has to flip sign.
    double lookahead_distance = lookahead_relative.Length();
    double curvature = 2.0 * side_distance / lookahead_distance;
    if (GetPathSegment().m_reverse) {
        curvature = -curvature;
    }
    sample.m_curvature_lookahead = curvature;

    // Method 1: Multiple the curvature by a gain. Work OK for a gain of 2, but oscillates a lot
    // curvature *= m_follower_parameters.m_curvature_gain;
    
    // Method 2: Add the path curvature
    curvature += path_curvature;

    sample.m_curvature = curvature;

    // Calculate the left and right velocity and acceleration to achieve the required
    // curvature and also the path velocity and path acceleration 
    double wheelbase_width_m = m_follower_parameters.m_wheelbase_width_m;
    double left_velocity = path_velocity * (2.0 - curvature*wheelbase_width_m)/2.0;
    double right_velocity = path_velocity * (2.0 + curvature*wheelbase_width_m)/2.0;
    double left_acceleration = path_acceleration * (2.0 - curvature*wheelbase_width_m)/2.0;
    double right_acceleration = path_acceleration * (2.0 + curvature*wheelbase_width_m)/2.0;

    // Do the PID follower calculation for each side of the robot to get the motor outputs
    double current_left_velocity;
    double current_right_velocity;
    drive_base->GetWheelVelocity(current_left_velocity, current_right_velocity);
    left_output = FollowSide(m_left_follower_state, left_velocity, left_acceleration,
                             current_left_velocity, sample.m_left_feed_forward);
    right_output = FollowSide(m_right_follower_state, right_velocity, right_acceleration,
                              current_right_velocity, sample.m_right_feed_forward);
    sample.m_left_velocity = left_velocity;
    sample.m_right_velocity = right_velocity;
    sample.m_left_acceleration = left_acceleration;
    sample.m_right_acceleration = right_acceleration;
    sample.m_current_left_velocity = current_left_velocity;
    sample.m_current_right_velocity = current_right_velocity;

	// Bound the output to the legal range of [-1, 1]. Do this by scaling both left and right
	// equally so that we don't get crazy extra turning.
	double max_speed = std::max(fabs(left_output), fabs(right_output));
	if (max_speed > 1.0) {
		left_output /= max_speed;
		right_output /= max_speed;
	}
}

Point2D PurePursuitFollower::GetClosestPoint(Point2D robot_position, double& path_velocity,
                                             double& path_acceleration, double& path_curvature) {
    // Zero the velocity and acceleration in case we don't find any values
    path_velocity = 0;
    path_acceleration = 0;
    path_curvature = 0;

    // Search for the closest point on the path starting at the previous closest point and
    // only searching forwards. This prevents accidentally turning around.
    int closest_index = -1;
    double closest_distance;
    Point2D closest_point;
    int path_index = m_closest_index;
    while (path_index < (int)m_path_points.size() - 1) {
        // Get the ends of the path segment to test
        const PathPoint& segment_pt1 = m_path_points[path_index];
        const PathPoint& segment_pt2 = m_path_points[path_index + 1];

        // Get the distance and relative position of the closest point on the path segment
        double relative;
        Point2D point_on_line = Geometry::ProjectPointOnToLine(robot_position, segment_pt1.m_position, segment_pt2.m_position, relative);
        if (0.0 <= relative && relative <= 1.0) {
            // Closest point is within the segment, so 'relative' and 'point_on_line' are OK
        }
        else if (relative < 0.0) {
            // Closest point is the beginning of the segment
            relative = 0.0;
            point_on_line = segment_pt1.m_position;
        } else {
            // Closest point is the end of the segment
            relative = 1.0;
            point_on_line = segment_pt2.m_position;
        }
        double distance = (robot_position - point_on_line).Length();

        // If this is the first test, or is closer than any other record the point as the current closest
        if (closest_index == -1 || distance < closest_distance) {
            closest_index = path_index;
            closest_distance = distance;
            closest_point = point_on_line;

            // Get the velocity and acceleration by linearly interpolating between the ends of the segment
            path_velocity = Lerp(segment_pt1.m_velocity, segment_pt2.m_velocity, relative);
            path_acceleration = Lerp(segment_pt1.m_acceleration, segment_pt2.m_acceleration, relative);
            path_curvature = Lerp(segment_pt1.m_curvature, segment_pt2.m_curvature, relative);
        } else if (distance > 10*closest_distance || distance > 2) {
            // If this segment is a long way further than the best then stop searching. This saves time
            // and also prevent accidentally picking up a later part of the path if it loops back on
            // itself (a figure-8 for example).
            break;
        } else if (path_index - m_closest_index > 5) {
            // Stop searching if we have advanced by more that 5 segments. This has been founc
            // to be effective in preventing the robot skipping to a later part of the path.
            // For 0.1 spacing and 20ms update 5 segments is 0.5/0.02 = 25m/s
            break;
        }
        path_index++;
    }

    // If no point was found just return the robot position (as this is safe) and record the segment as finished.
    // This should not be possible.
    if (closest_index == -1) {
        m_segment_finished = true;
        return robot_position;
    }

    //
    if (closest_index == (int)m_path_points.size() - 2) {
        m_segment_finished = true;
    }

    // Update the closest point index for next time and return the closest point
    m_closest_index = closest_index;
    return closest_point;
}

Point2D PurePursuitFollower::GetLookAheadPoint(Point2D robot_position, double closest_point_distance)
{
    // The lookahead distance must be greater than the distance to the path or it with most likely
    // not intersect with the path. Make sure the lookahead distance is greater that the closest point
    // distance by some margin. This margin is an important parameter. If it is not large enough then
    // the search may not find a lookahead point point and following the path will fail.
    double lookahead_distance = m_follower_parameters.m_lookahead_distance;
    if (lookahead_distance < closest_point_distance * 1.5) {
        lookahead_distance = closest_point_distance * 1.5;
    }

    // Search for the closest point on the path starting at the previous closest point and
    // only searching forwards. This prevents accidentally turning around.
    while (m_lookahead_index < (int)m_path_points.size() - 1) {
        // Get the ends of the path segment to test
        const PathPoint& segment_pt1 = m_path_points[m_lookahead_index];
        const PathPoint& segment_pt2 = m_path_points[m_lookahead_index + 1];

        // Get the distance and relative position of the closest point on the path segment
        double relative;
        Point2D point_on_line = Geometry::ProjectPointOnToLine(robot_position, segment_pt1.m_position, segment_pt2.m_position, relative);
        double distance = (robot_position - point_on_line).Length();

        double segment_length = (segment_pt1.m_position - segment_pt2.m_position).Length();

        double forward_distance = sqrt(lookahead_distance*lookahead_distance - distance*distance);

        double intersection_relative = relative + forward_distance/segment_length;

        if (0.0 <= intersection_relative && intersection_relative <= 1.0) {
            return Point2D::Lerp(segment_pt1.m_position, segment_pt2.m_position, intersection_relative);
        }

        m_lookahead_index++;
    }

    m_segment_finished = true;
    return robot_position;
}


//==========================================================================
// Following Calculations

double PurePursuitFollower::FollowSide(FollowerState& follower_state, double velocity, double acceleration,
                                       double current_velocity, double& feed_forward) {
	// NMBTODO Change follow side to track the error in the velocity

	// Calculate the error and accumulated total error
	double error = velocity - current_velocity;
	double total_error = follower_state.m_total_error + error;

	// Calculate the feed forward term. It contains 3 parts.
	//  1. A term proportional to the desired velocity
	//  2. A fixed term in the direction of the desired velocity
	//  3. A term proportional to the desired acceleration
	feed_forward = m_follower_parameters.m_kv * velocity +
                   m_follower_parameters.m_ka * acceleration;
    if (velocity != 0.0) {
        if (velocity > 0.0) {
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

void PurePursuitFollower::SetupSampleRecording() {
	// Clear the list of samples
	m_sample_list.clear();
    m_total_path_points.clear();

	// Reset and start the timer
	m_timer.Reset();
	m_timer.Start();
}

void PurePursuitFollower::WriteTestSampleToFile(const char* filename) {
    // Do nothing if we are not recording samples
    if (!GetRecordSamples()) return;

    
	std::ofstream results_file;
	results_file.open(filename, std::ios::out | std::ios::app);
	if (results_file.fail()) {
		std::cout << "PurePursuitFollower::WriteTestSampleToFile() - Failed to open result file\n";
		return;
	}

	// Write the parameters used for the test
	results_file << "\"Path\",\"" << GetRobotPath()->m_name << "\"\n";
	results_file << "\"Follower\",\"PurePursuitFollower\"\n";
	results_file << "\"kv\",\"" << m_follower_parameters.m_kv << "\"\n";
	results_file << "\"kv offset\",\"" << m_follower_parameters.m_kv_offset << "\"\n";
	results_file << "\"ka\",\"" << m_follower_parameters.m_ka << "\"\n";
	results_file << "\"kp\",\"" << m_follower_parameters.m_kp << "\"\n";
	results_file << "\"ki\",\"" << m_follower_parameters.m_ki << "\"\n";
	results_file << "\"kd\",\"" << m_follower_parameters.m_kd << "\"\n";
	results_file << "\"max velocity\",\"" << m_follower_parameters.m_max_velocity << "\"\n";
	results_file << "\"max aceleration\",\"" << m_follower_parameters.m_max_acceleration << "\"\n";
	results_file << "\"lookahead distance\",\"" << m_follower_parameters.m_lookahead_distance << "\"\n";
	results_file << "\"curvature gain\",\"" << m_follower_parameters.m_curvature_gain << "\"\n";
	results_file << "\"Wheelbase Width (m)\",\"" << m_follower_parameters.m_wheelbase_width_m << "\"\n";
	results_file << "\"Time Period (s)\",\"" << m_follower_parameters.m_period_s << "\"\n";
	results_file << "\"Time (s)\",\"" << (m_follower_parameters.m_period_s * m_sample_list.size()) << "\"\n";

   // Write the path points. Each parameter is on a separate line
	int total_points = m_total_path_points.size();
	results_file << "\"PathPointX\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_position.x;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathPointY\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_position.y;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathHeading\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_heading_degrees;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathDistance\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_distance;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathCurvature\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_curvature;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathMaxVelocity\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_max_velocity;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathVelocity\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_velocity;
	results_file << std::defaultfloat;
	results_file << "\n";
	results_file << "\"PathAcceleration\"";
	results_file << std::fixed;
	for (int i = 0; i < total_points; i++) results_file << "," << std::setprecision(3) << m_total_path_points[i].m_acceleration;
	results_file << std::defaultfloat;
	results_file << "\n";

    // Write the recorded sample data for the test. Each sample parameter is on a separate line
	int total_samples = m_sample_list.size();

	// Write the sample times in a single line
	results_file << "\"Time\"";
	results_file << std::fixed;
	for (int i = 0; i < total_samples; i++) results_file << "," << std::setprecision(3) << m_sample_list[i].m_time_s;
	results_file << std::defaultfloat;
	results_file << "\n";

	// Write the left and right wheel distances
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

    // Write the robot, closest and look ahead points
	results_file << "\"RobotX\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_robot_position.x;
	results_file << "\n";
	results_file << "\"RobotY\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_robot_position.y;
	results_file << "\n";
	results_file << "\"ClosestX\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_closest_point.x;
	results_file << "\n";
	results_file << "\"ClosestY\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_closest_point.y;
	results_file << "\n";
	results_file << "\"LookaheadX\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_lookahead_point.x;
	results_file << "\n";
	results_file << "\"LookaheadY\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_lookahead_point.y;
	results_file << "\n";

	results_file << "\"Curvature\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_curvature;
	results_file << "\n";
	results_file << "\"CurvatureLookahead\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_curvature_lookahead;
	results_file << "\n";
	results_file << "\"VelocityLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_left_velocity;
	results_file << "\n";
	results_file << "\"VelocityRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_right_velocity;
	results_file << "\n";
	results_file << "\"AccelerationLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_left_acceleration;
	results_file << "\n";
	results_file << "\"AccelerationRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_right_acceleration;
	results_file << "\n";
	results_file << "\"CurrentVelocityLeft\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_current_left_velocity;
	results_file << "\n";
	results_file << "\"CurrentVelocityRight\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_current_right_velocity;
	results_file << "\n";
	results_file << "\"ClosestIndex\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_closest_index;
	results_file << "\n";
	results_file << "\"LookaheadIndex\"";
	for (int i = 0; i < total_samples; i++) results_file << "," << m_sample_list[i].m_lookahead_index;
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
	// double left_error_total = 0.0;
	// double right_error_total = 0.0;
	// for (int i = 0; i < total_samples; i++) {
	// 	const Sample sample = m_sample_list[i];
	// 	left_error_total = abs(sample.m_left_distance_m - sample.m_left_point.m_distance);
	// 	right_error_total = abs(sample.m_right_distance_m - sample.m_right_point.m_distance);
	// }
	// std::cout << "Left Error Mean " << (left_error_total/total_samples) << "\n";
	// std::cout << "Right Error Mean " << (right_error_total/total_samples) << "\n";
}
