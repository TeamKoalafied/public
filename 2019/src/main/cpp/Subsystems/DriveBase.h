//==============================================================================
// DriveBase.h
//==============================================================================

#ifndef SRC_DRIVEBASE_H_
#define SRC_DRIVEBASE_H_

#include "TSingleton.h"
#include <frc/commands/Subsystem.h>

#include <string>

// Include in header because it uses many nested namespaces which are difficult for forward declaration.
#include <ctre/Phoenix.h>
#include <Commands/AutonomousCommand.h>
#include <Commands/VisionFindCube.h>


using namespace frc;

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
class DriveBase : public TSingleton<DriveBase>, public frc::Subsystem {
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
        ArcadeDrive(move, rotate);
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

    // Get the velocity in feet per second
    double GetVelocityFeetPerSecond();

    // Get the current heading from the Pigeon IMU device
    //
    // Returns the heading in degrees, or kHeadingError if the value cannot be read
    double GetPigeonHeading();

    // Get a stable heading from the Pigeon IMU device
    //
    // Returns the heading in degrees, or kHeadingError if the value cannot be read or is not stable
    double GetStablePigeonHeading();

    // get raw gyro to get angular velocity
    int GetPigeonRawGyro(double gyro_xyz_dps[3]);

    // Reset the Pigeon IMU device heading so that the current heading is 0 degrees
    void ResetPigeonHeading();


    //==========================================================================
    // Motor Performance

    double GetMotorVoltage();

private:
    //==========================================================================
    // Input Functions

    // Get the movement and rotation values from the joystick, including any speed
    // limiting and response curve shaping.
    //
    // move - returns the movement value between -1.0 and 1.0
    // rotate - returns the rotation value between -1.0 and 1.0
    void GetMovementFromJoystick(double& move, double& rotate);


    //==========================================================================
    // Drive Functions

    // Set the motor velocities for arcade drive with the given parameters
    //
    // move - Front/back proportional move value (+1 full forward to -1 full reverse)
    // rotate - Left/right proportional move value (-1 full left to +1 full right)
    void ArcadeDrive(double move, double rotate);

    // Calculate an adjusted rotation value to keep the robot driving in a straight lin``
    // move - Front/back proportional move value (+1 full forward to -1 full reverse)
    // rotate - Left/right proportional move value (-1 full left to +1 full right)
    void CalculateDriveStraightAdjustment(double move, double& rotate);


    void DriveToDistanceUltrasound();


    //==========================================================================
    // Calculation Functions

    // Convert a velocity from RMP to native TalonRX units
    //
    // velocity_rpm - velocity in RPM to convert
    //
    // Returns the velocity in TalonRX units (encoder counts per update period)
    double VelocityRmpToNative(double velocity_rpm);

    // Convert a velocity from native TalonRX units to RMP
    //
    // velocity_native - velocity in TalonRX units (encoder counts per update period) to convert
    //
    // Returns the velocity in RPM
    double VelocityNativeToRmp(double velocity_native);

    // Convert a encoder count of wheel rotation to a distance moved in inches
    //
    // encoder_count - encoder count from the drive base encoders (on axel shaft)
    //
    // Returns the distance in inches
    double EncoderToInches(int encoder_count);

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
    void DoTuningDriveSide(int joystick_axis, TalonSRX* speed_controller, const char* name, bool log_value);


    //==========================================================================
    // Member Variables

    TalonSRX* m_left_master_speed_controller;       // Left master Talon SRX speed controller
    TalonSRX* m_left_slave_speed_controller;        // Left slave Talon SRX speed controller
    TalonSRX* m_left_slave2_speed_controller;       // Left second slave Talon SRX speed controller
    TalonSRX* m_right_master_speed_controller;      // Right master Talon SRX speed controller
    TalonSRX* m_right_slave_speed_controller;       // Right slave Talon SRX speed controller
    TalonSRX* m_right_slave2_speed_controller;      // Right second slave Talon SRX speed controller

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


    int m_log_counter;                              // Counter for output log information

    // TODO Work out which auto commands should be available for driving and tidy them up
    VisionFindCube* m_find_cube_command;

    frc::Relay* m_vision_light_relay;


    //==========================================================================
    // Contants

    static constexpr double kHeadingError = 999.0; 	// Value indicating an error reading the heading from the Pigeon IMU
    static const int kTalonTimeoutMs = 10; 			// Default timeout for TalonSRX commands in milliseconds
    static const int kRunProfileSlotIdx = 0;		// PID profile slot id to use when running the robot
    static const int kTuneProfileSlotIdx = 1;		// PID profile slot id to use when tuning the controllers
    static const int kPidDefaultIdx = 0;			// PID id to use for all operations //TODO: Find correct pidIdx parameter
    static const int kGyroError = 999; 			    // Value indicating an error reading the gyro from the Pigeon IMU
    static const int kGyroTrue = 1; 				// Value indicating success reading the gyro from the Pigeon IMU
};

#endif /* SRC_DRIVEBASE_H_ */
