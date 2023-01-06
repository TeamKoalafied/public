//==============================================================================
// PathfinderController.h
//==============================================================================

#ifndef SRC_PATHFINDERCONTROLLER_H_
#define SRC_PATHFINDERCONTROLLER_H_

#include "PathController.h"
#include "../RobotPath/MechanismAction.h"
#include "../RobotPath/IMechanismController.h"
#include <string>
#include <vector>

struct Segment{
    double dt, x, y, position, velocity, acceleration, jerk, heading;
};

typedef struct {
    int initial_position, ticks_per_revolution;
    double wheel_circumference;
    double kp, ki, kd, kv, ka;
} EncoderConfig;

typedef struct {
    double last_error, heading, output;
    int segment, finished;
} EncoderFollower;


class PathfinderController : public PathController {
public:
	// Construction and Destruction
	PathfinderController(Segment* left_trajectory, Segment * right_trajectory, int trajectory_length, const char* name, bool own_trajectory);
	virtual ~PathfinderController();

	// Robot Setup
	void SetEncoderConfig(const EncoderConfig encoder_config);
	void SetEncoderCountPerRevolution(int encoder_count_per_revolution);
	void SetWheelCircumferenceM(double wheel_circumference_m);
	void SetPID(double p, double i, double d);

	// Trajectory Setup
	void InitialiseTrajectoryFromArrays(Segment* left_trajectory, Segment* right_trajectory, int trajectory_length, const char* name, bool own_trajectory);
	Segment* GetLeftTrajectory() const { return m_left_trajectory; }
	Segment* GetRightTrajectory() const { return m_right_trajectory; }

	// Mechanism Actions Setup
	void SetMechanismActions(IMechanismController* mechanism_controller, MechanismAction* mechanism_actions, int mechanism_actions_count);

	// Trajectory Following (from PathController)
	virtual void StartTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg);
	virtual const std::string* FollowTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg,
												double& left_output, double& right_output);
	virtual bool IsTrajectoryFinished();
	Segment* GetLeftSegment() const;
	Segment* GetRightSegment() const;

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

	double m_gyro_heading_offset_deg;			// Offset to add to the gyro to get the current heading in degrees

	EncoderConfig m_left_encoder_config;
	EncoderConfig m_right_encoder_config;

	const char* m_trajectory_name;
	Segment* m_left_trajectory;
	Segment* m_right_trajectory;
	int m_trajectory_length;
	bool m_own_trajectory;						// True if this class owns the trajectory buffers, and hence must delete them

	EncoderFollower m_left_follower;
	EncoderFollower m_right_follower;

	IMechanismController* m_mechanism_controller;
	MechanismAction* m_mechanism_actions;
	int m_mechanism_actions_count;
	int m_mechanism_actions_index;
	int m_period_counter;
};

#endif /* SRC_PATHFINDERCONTROLLER_H_ */
