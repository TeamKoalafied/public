//==============================================================================
// KoalafiedPathfinderController.cpp
//==============================================================================

#include "KoalafiedPathfinderController.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>


//==============================================================================
// Construction and Destruction

KoalafiedPathfinderController::KoalafiedPathfinderController(Segment* left_trajectory, Segment * right_trajectory,
															 int trajectory_length, const char* name, bool own_trajectory) :
	PathfinderController(left_trajectory, right_trajectory, trajectory_length, name, own_trajectory) {
}

KoalafiedPathfinderController::~KoalafiedPathfinderController() {
}


//==========================================================================
// Path Following Calculation

double KoalafiedPathfinderController::FollowEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
    if (follower->segment >= trajectory_length) {
        follower->finished = 1;
        follower->output = 0.0;
        Segment last = trajectory[trajectory_length - 1];
        follower->heading = last.heading;
        return 0.0;
    }

    const Segment& segment = trajectory[follower->segment];
    double distance_covered = ((double)encoder_tick - (double)c.initial_position) /  ((double)c.ticks_per_revolution);
    distance_covered = distance_covered * c.wheel_circumference;

	follower->finished = 0;
	double error = segment.position - distance_covered;
	double total_error = follower->output + error;
	double calculated_value = c.kp * error +
						      c.ki * total_error * segment.dt +
							  c.kd * ((error - follower->last_error) / segment.dt) +
							  (c.kv * segment.velocity + c.ka * segment.acceleration) +
							  (segment.velocity > 0.0 ? 0.05719 : -0.05719);

	// Steady state voltage response is
	// 		velocity(ft/s) = 1.1757 * voltage(V) - 0.8069
	// Hence
	//		voltage(V) = (velocity(ft/s) + 0.8069)/1.1757
	//
	//		output = voltage/12
	//             = (velocity(ft/s) + 0.8069)/14.1084             velocity(ft/s) = 3.2808*velocity(m/s)
	//             = (3.2808*velocity(m/s) + 0.8069)/14.1084
	//			   = 0.23254*velocity(m/s) + 0.05719

//    	std::cout << "distance_covered" << distance_covered << " s.position " << s.position << "\n";


	follower->last_error = error;
	follower->heading = segment.heading;
	follower->output = total_error;
	follower->segment = follower->segment + 1;
	return calculated_value;
}

std::string KoalafiedPathfinderController::FollowerName() {
	return "Koalafied";
}


