//==============================================================================
// DriveBase.h
//==============================================================================

#ifndef SRC_DRIVEBASE_H_
#define SRC_DRIVEBASE_H_

#include "../TSingleton.h"
#include "../HapticController.h"
#include "../Commands/RobotPath/IPathDriveBase.h"
#include <frc/commands/Subsystem.h>

#include <string>

// Include in header because it uses many nested namespaces which are difficult for forward declaration.
#include <ctre/Phoenix.h>
#include "../Commands/AutonomousCommand.h"
#include "../Commands/VisionFindCube.h"
#include "../Commands/VisionFindTarget.h"




namespace frc
{
class Joystick;
class Talon;
class Ultrasonic;
}

// DriveBase controls everything to do with the robot drive base, including:
// - Motor speed controllers
// - Pigeon IMU
// - Joystick object
//
// Implements the 'Cheezy' drive system.
class DriveBase : public TSingleton<DriveBase>, public frc::Subsystem, public IPathDriveBase {
public:
    //==========================================================================
    // Construction

    // Constructor
    DriveBase();

    // Destructor
    virtual ~DriveBase();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the drive base for robot operation, with encoders, slave drive etc.
    void Setup();

    // Reset the drive base to the default state
    void ClearState();

    // Shutdown the drive base
    void Shutdown();


    //==========================================================================
    // Operation

    // Reset the joystick state (prevents spurious button events)
    void ResetJoystickState();

    // Do joystick control of the drive base using the cheezy drive method (triggers
    // control forward/backwards and the left stick steers left/right).
    void DoCheezyDrive();

    // Set up heading for DriveStraightAdjustment
    //
    // heading - Heading to drive to in degrees
    //
    // This function is used for autonomous driving, when a heading to drive to is known.
    void StartDrivingStraight(double heading);

    // Drive straight, at the given speed
    //
    // velocity_feet_per_second - Velocity to drive at in feet/s
    // rotate - Left/right proportional rotate value (-1 full left to +1 full right)
    void Drive(double velocity_feet_per_second, double rotate);

    // Drive straight for the given distance in inches, using Talon Motion Magic
    //
    // distance_inch - distance to drive in inches
    void DriveDistance(double distance_inch, double velocity_feet_per_second);

    // Temporary arcade drive function for the vision command the uses proportional
    // movement rather than a real world velocity.
    void ArcadeDriveForVision(double move, double rotate) { // TODO Change vision to use real velocity
        Sample sample;
        ArcadeDrive(move, rotate, sample);
    }

    // Do tank drive in open loop mode
    //
    // left_output - proportional signal for the left motor in the range -1 to + 1
    // right_output - proportional signal for the right motor in the range -1 to + 1
    void TankDriveOpenLoop(double left_output, double right_output);

    // Stops the drive base
    void Stop();

    // Set the brake mode for the drive base motors
    //
    // brake - Whether to set to brake (true) or cruise (false)
    void SetBrakeMode(bool brake);


    //==========================================================================
    // Distance and Heading

    // Reset the distance measurement
    void ResetDistance();

    // Get the distance the robot has moved since the last distance reset
    double GetDistanceInch();

    // Get the distance measured by the left an right encoders, in native encoder units
    //
    // left_encoder - Returns the left encoder value
    // right_encoder - Returns the right encoder value
    void GetEncoderDistances(int& left_encoder, int& right_encoder);

    // Get the distance travelled by the left and right wheels, as measured by the encoders,
    // but converted into metres
    //
    // left_distance_m - Returns the left distance in metres
    // right_distance_m - Returns the right distance in metres
    void GetWheelDistancesM(double& left_distance_m, double& right_distance_m);

    // Get velocity of the left and right wheels
    //
    // left_velocity - Returns the left wheel velocity
    // right_velocity - Returns the right wheel velocity
    virtual void GetWheelVelocity(double& left_velocity, double& right_velocity);

    // Get the velocity in feet per second
    double GetVelocityFeetPerSecond();

    // Value indicating an error reading the heading from the Pigeon IMU
    static constexpr double kHeadingError = 999.0; 

    // Get the current heading from the Pigeon IMU device
    //
    // Returns the heading in degrees, or kHeadingError if the value cannot be read. Positive is anticlockwise.
    double GetPigeonHeading();

    // Get a stable heading from the Pigeon IMU device
    //
     // Returns the heading in degrees, or kHeadingError if the value cannot be read or is not stable. Positive is anticlockwise.
    double GetStablePigeonHeading();

    // Reset the Pigeon IMU device heading so that the current heading is 0 degrees
    void ResetPigeonHeading();

    // Reset the dead reconning position. A position and heading can be specified.
    //
    // position_x_inch - The x position in inches
    // position_y_inch - The y position in inches
    // heading_degrees - The 'position' heading in degrees (this is the not the pigeon heading)
   void ResetPosition(double position_x_inch = 0.0, double position_y_inch = 0.0, double heading_degrees = 0.0);

    // Get the dead reconning position, in inches
    //
    // position_x_inch - Returns the x position in inches
    // position_y_inch - Returns the y position in inches
    // heading_degrees - Returns the 'position' heading in degrees (this is the not the pigeon heading)
    //
    // Dead recconing used standard mathematical convention for x and y axis. Zero heading is along the x-axis
    // and angles increase in the anitclockwise direction.
    void GetPositionInch(double& position_x_inch, double& position_y_inch, double& heading_degrees);

    // Get the dead reconning position, in metres
    //
    // position_x_inch - Returns the x position in metres
    // position_y_inch - Returns the y position in metres
    // heading_degrees - Returns the 'position' heading in degrees (this is the not the pigeon heading)
    //
    // Dead recconing used standard mathematical convention for x and y axis. Zero heading is along the x-axis
    // and angles increase in the anitclockwise direction.
    void GetPositionM(double& position_x_m, double& position_y_m, double& heading_degrees);


    //==========================================================================
    // Motor Performance

    // Get the Measured applied motor voltage (V)
    double GetMotorVoltage();

    // Get the total motor current (A). Absolute current from all motors summed. 
    double GetMotorCurrent();

private:
    //==========================================================================
	// Private Nested Types

	// Sample data recorded during a test
	struct Sample
	{
        double m_move_input;            // Raw move input value (+ve if right trigger -ve if left trigger)
        double m_rotate_input;          // Raw rotate joystick value
        double m_move;                  // Move value after power adjustment
        double m_rotate;                // Rotate value after power adjustment and rotate scaling
        double m_stable_heading;        // Stable version of the pigeon heading (can be kHeadingError)
        bool   m_driving_straight;      // Whether drive straight is currently occuring
        double m_rotate_straight;       // Rotate value after drive straight adjustment
		double m_left_output;			// Proportional output to the left motor [-1, 1]
		double m_right_output;			// Proportional output to the right motor [-1, 1]
		double m_left_distance_m;		// Measured left distance in metres, relative to path s
		double m_right_distance_m;		// Measured right error in metres
		double m_gyro_heading_deg;		// Measured gyro heading in degrees
		double m_robot_position_x;	    // X coordinate of the position of the robot
		double m_robot_position_y;	    // Y coordinate of the position of the robot
	};

    //==========================================================================
    // Input Functions

    // Get the movement and rotation values from the joystick, including any speed
    // limiting and response curve shaping.
    //
    // move - returns the movement value between -1.0 and 1.0
    // rotate - returns the rotation value between -1.0 and 1.0
    // sample - Sample to record log information in
    void GetMovementFromJoystick(double& move, double& rotate, Sample& sample);


    //==========================================================================
    // Drive Functions

    // Set the motor velocities for arcade drive with the given parameters
    //
    // move - Front/back proportional move value (+1 full forward to -1 full reverse)
    // rotate - Left/right proportional move value (-1 full left to +1 full right)
    // sample - Sample to record log information in
    void ArcadeDrive(double move, double rotate, Sample& sample);

    // Calculate an adjusted rotation value to keep the robot driving in a straight lin``
    // move - Front/back proportional move value (+1 full forward to -1 full reverse)
    // rotate - Left/right proportional move value (-1 full left to +1 full right)
    // sample - Sample to record log information in
    void CalculateDriveStraightAdjustment(double move, double& rotate, Sample& sample);


    void DriveToDistanceUltrasound();


    //==========================================================================
    // Calculation Functions

    // Convert a velocity from RMP to native TalonRX units
    //
    // velocity_rpm - velocity in RPM to convert
    //
    // Returns the velocity in TalonRX units (encoder counts per update period)
//    double VelocityRmpToNative(double velocity_rpm);

    // Convert a velocity from native TalonRX units to RMP
    //
    // velocity_native - velocity in TalonRX units (encoder counts per update period) to convert
    //
    // Returns the velocity in RPM
//    double VelocityNativeToRmp(double velocity_native);

    // Convert a encoder count of wheel rotation to a distance moved in inches
    //
    // encoder_count - encoder count from the drive base encoders (on axel shaft)
    //
    // Returns the distance in inches
    double EncoderToInches(double encoder_count);

    // Convert a distance moved in inches to an encoder count
    //
    // distance_inches - distance in inches
    //
    // Returns the encoder count
    double InchesToEncoder(int distance_inches);


    //==========================================================================
    // Motor Tuning

    // Controls the motors and log values for calculating PID parameters
    void DoTuningDrive();

    // DoTuningDrive for one side (left/right) of the drive base
    //
    // joystick_axis - the joystick axis for controlling the speed on this side of the of the drive base
    // speed_controller - the master speed controller for this side of the of the drive base
    // name - the name of this side of the of the drive base
    // log_value - whether to log values on this update
    void DoTuningDriveSide(int joystick_axis, TalonFX* speed_controller, const char* name, bool log_value);
    //==========================================================================
    // Manual Drive Logging

 	// Setup the recording of 'sample' data for testing
	void SetupSampleRecording();

	// Write the recorded test samples to a file. Does nothing if 'm_record_samples' is false.
    //
    // filename - Path of the file to write ot
	void WriteTestSampleToFile(const char* filename);
   


    //==========================================================================
    // Member Variables

    TalonFX* m_left_master_speed_controller;       // Left master Talon SRX speed controller
    TalonFX* m_left_slave_speed_controller;     // Left second slave Talon SRX speed controller
    TalonFX* m_right_master_speed_controller;      // Right master Talon SRX speed controller
    TalonFX* m_right_slave_speed_controller;   // Right second slave Talon SRX speed controller

    frc::Joystick* m_joystick;                      // Joystick
    PigeonIMU* m_pigen_imu;                         // Pigeon IMU (inertial measurement unit)

    frc::Ultrasonic* m_forward_ultrasonic;          // Forward facing ultrasound. 
    frc::Ultrasonic* m_backward_ultrasonic;         // Backward facing ultrasound. 

    bool m_driving_straight;                        // Whether we are currently trying to drive straight
    double m_straight_heading;                      // Heading we are trying to drive straight along (valid if m_driving_straight is true)
    static const int TOTAL_HEADINGS = 25;     		// Number of headings to determine 'stability' over
    double m_heading_buffer[TOTAL_HEADINGS];   		// List of recorded headings
    int m_heading_buffer_index;                     // Index of the next heading to record

    static const int TOTAL_VELOCITIES = 25;   		// Number of velocities to determine acceleration over
    double m_velocity_buffer[TOTAL_VELOCITIES]; 	// List of recorded velocities
    int m_velocity_buffer_index;                    // Index of the next velocities to record

    double m_position_x_inch;                       // Dead reconning x position in inches
    double m_position_y_inch;                       // Dead reconning y position in inches
    double m_position_heading_offset;               // Offset added to pigeon heading to get dead reconning heading
    int m_position_last_left_encoder;               // Dead reconning last left encoder position
    int m_position_last_right_encoder;              // Dead reconning last right encoder position

    int m_log_counter;                              // Counter for output log information

    // TODO Work out which auto commands should be available for driving and tidy them up
    VisionFindCube* m_find_cube_command;

    frc::Relay* m_vision_light_relay;
    std::vector<Sample> m_sample_list;			    // List of data samples recorded during manual driving

    HapticController* m_haptic_controller;

    //==========================================================================
    // Contants

    static const int kTalonTimeoutMs = 10; 			// Default timeout for TalonSRX commands in milliseconds
    static const int kRunProfileSlotIdx = 0;		// PID profile slot id to use when running the robot
    static const int kTuneProfileSlotIdx = 1;		// PID profile slot id to use when tuning the controllers
    static const int kPidDefaultIdx = 0;			// PID id to use for all operations //TODO: Find correct pidIdx parameter
    static const int kGyroError = 999; 			    // Value indicating an error reading the gyro from the Pigeon IMU
    static const int kGyroTrue = 1; 				// Value indicating success reading the gyro from the Pigeon IMU
};

#endif /* SRC_DRIVEBASE_H_ */
