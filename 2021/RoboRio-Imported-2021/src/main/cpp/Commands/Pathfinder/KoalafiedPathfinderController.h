//==============================================================================
// KoalafiedPathfinderController.h
//==============================================================================

#ifndef SRC_KOALAFIEDPATHFINDERCONTROLLER_H_
#define SRC_KOALAFIEDPATHFINDERCONTROLLER_H_

#include "PathfinderController.h"
#include <string>

class KoalafiedPathfinderController : public PathfinderController {
public:
	// Construction and Destruction
	KoalafiedPathfinderController(Segment* left_trajectory, Segment * right_trajectory, int trajectory_length,
								  const char* name, bool own_trajectory);
	virtual ~KoalafiedPathfinderController();

protected:

    //==========================================================================
	// Path Following Calculation

	virtual double FollowEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick);
	virtual std::string FollowerName();

private:

    //==========================================================================
	// Member Variables

};

#endif /* SRC_KOALAFIEDPATHFINDERCONTROLLER_H_ */
