//==============================================================================
// SwerveTrajectoryGenerator.cpp
//==============================================================================

#include "SwerveTrajectoryGenerator.h"

#include "Vector2D.h"
#include "../util/KoalafiedUtilities.h"

#include <units/length.h>

void SwerveTrajectoryGenerator::Generate(SwerveTrajectory& trajectory, const std::vector<Bezier3_2D<units::meter_t> >& bezier_list,
 const std::vector<units::degree_t>& angles, const Parameters& parameters) {

    // Vector2D<units::meter_t> position1(1_m, 1_m);
    // Vector2D<units::meter_t> position2(1_m, -1_m);

    // Vector2D<units::meter_t> position3 = position1 + position2;

    // position3 = position1 - position2;
    // position3 = 0.5 * position1 + 0.5 * position2;
    // units::meter_t  length = position1.Length();

	// Get the lengths of all the beziers in the current path segment
	int total_beziers = bezier_list.size();
	std::vector<units::meter_t> segment_lengths(total_beziers);
	units::meter_t total_length = 0_m;
	for (int i = 0; i < total_beziers; i++) {
		segment_lengths[i] = bezier_list[i].Length();
		total_length += segment_lengths[i];
	}
    if (total_beziers == 0) return;

    // Calculate the total number of points for the required spacing and create space
    // for them, plus the 'extension' point.
    units::meter_t point_spacing = parameters.m_path_point_spacing;
	int total_points = (int)ceil((double)(total_length / point_spacing)) + 1;
	trajectory.Reset(total_points + 1);

    std::vector<int> bezier_start_index(total_beziers + 1);
    bezier_start_index[0] = 0;
    bezier_start_index[total_beziers] = total_points - 1;

	// Loop through the array setting up the points
	int bezier_index = 0;
	units::meter_t total_pos = 0_m;
	units::meter_t bezier_pos = 0_m;
	for (int i = 0; i < total_points; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);

        // Record the distance along the path
        path_point.m_distance = total_pos;

        // Calculate the position by measuring along the arc of the current bezier segment
        const Bezier3_2D<units::meter_t>& bezier = bezier_list[bezier_index];
        double u;
        path_point.m_position = bezier.ArcLengthPoint(bezier_pos, u);

        // Get the derivative of the bezier and convert it to a heading in degrees, normalised to [0, 360)
        Vector2D<units::meter_t> direction = bezier.Derivative(u);
        direction.Normalize();
        path_point.m_velocity_vector.x = units::meters_per_second_t(direction.x.value());
        path_point.m_velocity_vector.y = units::meters_per_second_t(direction.y.value());

        // Use the derivative of the bezier to calculate a heading in degrees, normalised to [0, 360)
        // NOT REQUIRED       
        units::degree_t heading_degrees = units::math::atan2(direction.y, direction.x);
        while (heading_degrees >= 360.0_deg) heading_degrees -= 360.0_deg;
        while (heading_degrees <    0.0_deg) heading_degrees += 360.0_deg;
        path_point.m_heading_degrees = heading_degrees;

        // Get the curvature and record the maximum velocity. The maximum velocity is reduced if the
        // curvature is too large. Note that curvature is signed (+ve left)
        path_point.m_curvature = bezier.Curvature(u);
        path_point.m_max_velocity = parameters.m_max_velocity;
        if (path_point.m_curvature != 0.0) {
            // NMBTODO Consider determining the max velocity from a maximum centripetal acceleration
            // v = sqrt(a/C)
            units::meters_per_second_t curve_max_velocity = parameters.m_max_velocity_curve / fabs(path_point.m_curvature);
            if (path_point.m_max_velocity > curve_max_velocity) {
                path_point.m_max_velocity = curve_max_velocity;
            }
        }

        // TODO: Determine the maximum velocity so that the required angle change can be made and
        // clip the maximum velocity to this

        // Update the total position and the position relative to the current bezier segment.
        total_pos += point_spacing;
        bezier_pos += point_spacing;

        // If we have advanced past the end of the current bezier segment move on to the next one, but
        // don't go past the last one, just move to its exact length instead.
        if (bezier_pos > segment_lengths[bezier_index]) {
            bezier_pos -= segment_lengths[bezier_index];
            if (bezier_index < total_beziers - 1) {
                bezier_index++;
                bezier_start_index[bezier_index] = i;
            }
            else {
                bezier_pos = segment_lengths[bezier_index];
            }
        }
	}

    // Start the path with zero velocity and move forward along it accelerating up to the maximum
    // allowed velocity for each point.
    units::meters_per_second_t velocity = 0.0_mps;
    trajectory.GetPoint(0).m_velocity = 0_mps;
    trajectory.GetPoint(0).m_velocity_vector.Set(0_mps, 0_mps);
 	for (int i = 1; i < total_points; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);

        units::meter_t distance = path_point.m_distance - trajectory.GetPoint(i - 1).m_distance;
        velocity = units::math::sqrt(2*distance * parameters.m_max_acceleration + velocity*velocity);
        if (velocity > path_point.m_max_velocity) {
            velocity = path_point.m_max_velocity;
        }
        path_point.m_velocity = velocity;

        // path_point.m_velocity_vector.x = velocity * path_point.m_velocity_vector.x.value();
        // path_point.m_velocity_vector.y = velocity * path_point.m_velocity_vector.y.value();
    }

    // End the path with zero velocity and move backwards along it accelerating up to the current
    // velocity from the foward sweep for each point.
    velocity = 0_mps;
    trajectory.GetPoint(total_points - 1).m_velocity = 0_mps;
 	for (int i = total_points - 2; i >= 0; i--) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);

        units::meter_t distance = trajectory.GetPoint(i + 1).m_distance - path_point.m_distance;
        velocity = units::math::sqrt(2*distance * parameters.m_max_acceleration + velocity*velocity);
        if (velocity > path_point.m_velocity) {
            velocity = path_point.m_velocity;
        }
        path_point.m_velocity = velocity;

        // path_point.m_velocity_vector.x = velocity * path_point.m_velocity_vector.x.value();
        // path_point.m_velocity_vector.y = velocity * path_point.m_velocity_vector.y.value();
    }

 	for (int i = 0; i < total_points; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);
        path_point.m_velocity_vector.x = path_point.m_velocity * path_point.m_velocity_vector.x.value();
        path_point.m_velocity_vector.y = path_point.m_velocity * path_point.m_velocity_vector.y.value();
    }


    // Calculate the acceleration for each point and the time to traverse the path
    units::second_t path_time = 0_s;
 	for (int i = 0; i < total_points - 1; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);
		SwerveTrajectory::Point& next_path_point = trajectory.GetPoint(i + 1);

        path_point.m_time = path_time;

        units::meters_per_second_t velocity = path_point.m_velocity;
        units::meters_per_second_t final_velocity = next_path_point.m_velocity;
        units::meter_t distance = next_path_point.m_distance - path_point.m_distance;

        //path_point.m_acceleration = (final_velocity * final_velocity - velocity * velocity)/(2*distance);

    //    units::meters_per_second_squared_t acceleration = (final_velocity * final_velocity - velocity * velocity)/(2*distance);

        units::second_t segment_time = distance / (0.5 * (velocity + final_velocity));
        path_point.m_acceleration = (final_velocity  - velocity)/segment_time;
//        path_point.m_acceleration_vector = (next_path_point.m_velocity_vector - path_point.m_velocity_vector)/segment_time;
        path_point.m_acceleration_vector.x = (next_path_point.m_velocity_vector.x - path_point.m_velocity_vector.x)/segment_time;
        path_point.m_acceleration_vector.y = (next_path_point.m_velocity_vector.y - path_point.m_velocity_vector.y)/segment_time;



        path_time += segment_time;
    }
    
    // Calculate the angle for each point
	bezier_index = 0;
 	for (int i = 0; i < total_points; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);

        int start_index = bezier_start_index[bezier_index];
        int end_index = bezier_start_index[bezier_index + 1];
		SwerveTrajectory::Point& start_point = trajectory.GetPoint(start_index);
		SwerveTrajectory::Point& end_point = trajectory.GetPoint(end_index);

        
        units::degree_t angle0 = angles[bezier_index];
        units::degree_t angle1 = angles[bezier_index + 1];
        // units::second_t t0 = start_point.m_time;
        // units::second_t t1 = end_point.m_time;
        // units::second_t t = path_point.m_time;
//        path_point.m_rotation_degrees = angle0 + ((t - t0)/(t1 - t0)) * KoalafiedUtilities::NormaliseAngleDiff(angle1, angle0);
        units::meter_t d0 = start_point.m_distance;
        units::meter_t d1 = end_point.m_distance;
        units::meter_t d = path_point.m_distance;
        path_point.m_rotation_degrees = angle0 + ((d - d0)/(d1 - d0)) * KoalafiedUtilities::NormaliseAngleDiff(angle1, angle0);
    }

 	for (int i = 1; i < total_points; i++) {
		SwerveTrajectory::Point& path_point = trajectory.GetPoint(i);
		SwerveTrajectory::Point& previous_path_point = trajectory.GetPoint(i - 1);

        path_point.m_rotational_velocity_degrees = (path_point.m_rotation_degrees - previous_path_point.m_rotation_degrees)/
                                                   (path_point.m_time - previous_path_point.m_time);
    }
    trajectory.GetPoint(total_points - 1).m_rotational_velocity_degrees = 0.0_deg_per_s;


    // Add an extra 'extension' point to the end of the points list. This gives a long straight segment
    // where we can look for the 'lookahead' point as we come to the end of the path.
	units::meter_t extension_distance = 1_m;
    SwerveTrajectory::Point& last_point = trajectory.GetPoint(total_points - 1);
	units::radian_t extension_heading_radians = last_point.m_heading_degrees;
	Vector2D<units::meter_t> extension_position = last_point.m_position;
	extension_position.x += extension_distance * units::math::cos(extension_heading_radians);
	extension_position.y += extension_distance * units::math::sin(extension_heading_radians);
    SwerveTrajectory::Point& extension_point = trajectory.GetPoint(total_points);

	extension_point.m_time = last_point.m_time;

	extension_point.m_position = extension_position;
	extension_point.m_velocity_vector = Vector2D<units::meters_per_second_t>();
	extension_point.m_acceleration_vector = Vector2D<units::meters_per_second_squared_t>();

    extension_point.m_rotation_degrees = last_point.m_rotation_degrees;
    extension_point.m_rotational_velocity_degrees = 0.0_deg_per_s;
    extension_point.m_rotational_acceleration_degrees = 0.0_deg_per_s_sq;

	extension_point.m_heading_degrees = last_point.m_heading_degrees;
	extension_point.m_distance = last_point.m_distance + extension_distance;
	extension_point.m_velocity = 0_mps;
	extension_point.m_acceleration = 0_mps_sq;
	extension_point.m_curvature = 0.0;

	extension_point.m_max_velocity = 0_mps;

    trajectory.SetTime(path_time);
}