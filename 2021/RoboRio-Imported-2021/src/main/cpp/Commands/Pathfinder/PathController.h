//==============================================================================
// PathController.h
//==============================================================================

#ifndef SRC_PATHCONTROLLER_H_
#define SRC_PATHCONTROLLER_H_

#include <frc/Timer.h>
#include <vector>
#include <string>

// PathController is a base class for controllers that drive the robot along a path.
// This class provides a uniform interface that the DrivePathfinder command can use
// and handles recording data samples about the path and writing them to a file.
class PathController {
public:
	// Construction and Destruction
	PathController();
	virtual ~PathController();

	// Trajectory Following
	virtual void StartTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg) = 0;
	virtual const std::string* FollowTrajectory(int left_encoder, int right_encoder, double gyro_heading_deg,
											    double& left_output, double& right_output) = 0;
	virtual bool IsTrajectoryFinished() = 0;

	// Test Logging
	void WriteTestSampleToFile();
	bool GetRecordSamples() const { return m_record_samples; }
	void SetRecordSamples(bool record_samples) { m_record_samples = record_samples; }

protected:
    //==========================================================================
	// Sample Recording

	void SetupSampleRecording();
	void RecordSample(double left_distance_m, double right_distance_m, double gyro_heading_deg,
					  double left_output, double right_output, const std::string* mechanism_action);
	void WriteSampleHeading(std::ofstream& results_file);

private:
    //==========================================================================
	// Nested Types

	// Sample data recorded during a test
	struct Sample
	{
		double m_time_s;				// Time in seconds the sample was taken at relative to the start of the test
		double m_left_distance_m;		// Measured left distance in metres, relative to path s
		double m_right_distance_m;		// Measured right error in metres
		double m_gyro_heading_deg;		// Measured gyro heading in degrees
		double m_left_output;			// Proportional output to the left motor [-1, 1]
		double m_right_output;			// Proportional output to the right motor [-1, 1]
		const std::string* m_mechanism_action; // Mechanism action performed (NULL for none)
	};


    //==========================================================================
	// Member Variables

    std::vector<Sample> m_sample_list;			// List of data samples recorded during the test
	bool m_record_samples;						// Whether samples are recorded (default) or discarded
	frc::Timer m_timer;
};

#endif /* SRC_PATHCONTROLLER_H_ */
