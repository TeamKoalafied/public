//==============================================================================
// PathfinderController.h
//==============================================================================

#ifndef SRC_PATHFINDERCONTROLLER_H_
#define SRC_PATHFINDERCONTROLLER_H_

#include "../PathController.h"
#include <pathfinder.h>
//#include <Timer.h>
#include <vector>


class PathfinderController : public PathController {
public:
	// Construction and Destruction
	PathfinderController(Segment* left_trajectory, Segment * right_trajectory, int trajectory_length, const char* name);
	virtual ~PathfinderController();

	// Robot Setup
	void SetEncoderCountPerRevolution(int encoder_count_per_revolution);
	void SetWheelCircumferenceM(double wheel_circumference_m);
	void SetPID(double p, double i, double d);

	// Trajectory Setup
	void InitialiseTrajectoryFromArrays(Segment* left_trajectory, Segment* right_trajectory, int trajectory_length, const char* name);

	// Trajectory Following (from PathController)
	virtual void StartTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg);
	virtual void FollowTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg, double& left_output, double& right_output);
	virtual bool IsTrajectoryFinished();

protected:
    //==========================================================================
	// Sample Recording(from PathController)

	virtual void WriteSampleHeading(std::ofstream& results_file);

    //==========================================================================
	// Path Following Calculation

	virtual double FollowEncoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) = 0;
	virtual std::string FollowerName() = 0;

private:

    //==========================================================================
	// Member Variables

	double m_delta_t_sec;						// Time delta in seconds
	double m_initial_gyro_heading_deg;			// Initial heading of the gyro in degrees

	EncoderConfig m_left_encoder_config;
	EncoderConfig m_right_encoder_config;

	const char* m_trajectory_name;
	Segment* m_left_trajectory;
	Segment* m_right_trajectory;
	int m_trajectory_length;

	EncoderFollower m_left_follower;
	EncoderFollower m_right_follower;
};

#endif /* SRC_PATHFINDERCONTROLLER_H_ */
