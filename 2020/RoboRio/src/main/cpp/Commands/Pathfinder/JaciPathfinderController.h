//==============================================================================
// JaciPathfinderController.h
//==============================================================================

#ifndef SRC_JACIPATHFINDERCONTROLLER_H_
#define SRC_JACIPATHFINDERCONTROLLER_H_

#include "PathfinderController.h"
#include <string>


class JaciPathfinderController : public PathfinderController {
public:
	// Construction and Destruction
	JaciPathfinderController(Segment* left_trajectory, Segment * right_trajectory, int trajectory_length,
							 const char* name, bool own_trajectory);
	virtual ~JaciPathfinderController();

protected:

    //==========================================================================
	// Path Following Calculation

	virtual double FollowEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick);
	virtual std::string FollowerName();

private:

    //==========================================================================
	// Member Variables

};

#endif /* SRC_JACIPATHFINDERCONTROLLER_H_ */
