//==============================================================================
// JaciPathfinderController.cpp
//==============================================================================

#include "JaciPathfinderController.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>


double my_pathfinder_follow_encoder2(EncoderConfig c, EncoderFollower *follower, Segment s, int trajectory_length, int encoder_tick) {
    double distance_covered = ((double)encoder_tick - (double)c.initial_position) /  ((double)c.ticks_per_revolution);
    distance_covered = distance_covered * c.wheel_circumference;

    if (follower->segment < trajectory_length) {
        follower->finished = 0;
        double error = s.position - distance_covered;
        double calculated_value = c.kp * error +
                                  c.kd * ((error - follower->last_error) / s.dt) +
                                  (c.kv * s.velocity + c.ka * s.acceleration);

//    	std::cout << "distance_covered" << distance_covered << " s.position " << s.position << "\n";


        follower->last_error = error;
        follower->heading = s.heading;
        follower->output = calculated_value;
        follower->segment = follower->segment + 1;
        return calculated_value;
    } else {
        follower->finished = 1;
        return 0.0;
    }
}

double my_pathfinder_follow_encoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
    int segment = follower->segment;
    if (segment >= trajectory_length) {
        follower->finished = 1;
        follower->output = 0.0;
        Segment last = trajectory[trajectory_length - 1];
        follower->heading = last.heading;
        return 0.0;
    } else {
        return my_pathfinder_follow_encoder2(c, follower, trajectory[segment], trajectory_length, encoder_tick);
    }
}


//==============================================================================
// Construction and Destruction

JaciPathfinderController::JaciPathfinderController(Segment* left_trajectory, Segment * right_trajectory,
                                                   int trajectory_length, const char* name, bool own_trajectory) :
	PathfinderController(left_trajectory, right_trajectory, trajectory_length, name, own_trajectory) {
}

JaciPathfinderController::~JaciPathfinderController() {
}


//==========================================================================
// Path Following Calculation

double JaciPathfinderController::FollowEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
	return my_pathfinder_follow_encoder(c, follower, trajectory, trajectory_length, encoder_tick);
}

std::string JaciPathfinderController::FollowerName() {
	return "Jaci";
}


