//==============================================================================
// Elevator.h
//==============================================================================

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_

#include "TSingleton.h"

class DriveElevator;
class DriveArm;
class EjectClaw;
#include <string>
#include <Solenoid.h>
#include <Timer.h>
#include "RobotConfiguration.h"
#include <Commands/Subsystem.h>

#include <ctre/Phoenix.h>


namespace frc
{
class Joystick;
class Talon;
}

// The elevator subsystem controls the mechanism for grabbing and lifting cubes.
// It consists of the following parts:
//
// 1. The claw - this is the intake/gripper mechanism. It has roller motors on arms
//    controlled by pneumatic pistons. When grabbing a cube we can detect that the
//    cube is present by looking for an increase in the roller motor current.
// 2. The arm - this is a 4 bar linkage that connects the claw to the lift. In the
//    up position it is inside the frame perimeter to make the robot legal for starting
//    and is also high enough to make the lift reach the switch in the highest position.
// 3. The lift - this is a two stage lift control by a single motor, with limit switches
//    that are wired directly to the Talon SRX speed controller.
//
// These three mechanism are grouped as a single subsystem because they cannot be used
// independently, only by a single command at a time. Also grouping together makes certain
// control logic easier.
class Elevator : public TSingleton<Elevator>, public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
	Elevator();

	// Destructor
	virtual ~Elevator();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the elevator subsystem
    void Setup();

    // Reset the elevator to the default state
    void ClearState();

    // Shutdown the elevator subsystem
    void Shutdown();


    //==========================================================================
    // Joystick Operation

    // Reset the joystick state (prevents spurious button events)
    void ResetJoystickState();

    // Do joystick control of the elevator using the operator joystick
    void DoOperatorDrive();


    //==========================================================================
    // Claw Operation

    // Open the claw
    void OpenClaw();

    // Close the claw
    void CloseClaw();

    // Get whether the claw is open
    bool IsClawOpen();

    // State of the rollers
    enum class RollerState {
    	Grabbing,		// The rollers are running to grab the cube
		Ejecting,		// The rollers are running to eject the cube
		Stopped			// The rollers are stopped
    };

    // Run the claw roller to grab the cube, with an optional timeout. Note that
    // if there is high current indicating that the cube has been grabbed then
    // the rollers will stop automatically.
    //
    // timeout_s - Time in seconds before stopping. -1 indicates no timeout.
    void RunClawRollerGrab(int timeout_s);

    // Run the claw roller to eject the cube, with an optional timeout. Note that
    // if there is no high current stopping.
    //
    // timeout_s - Time in seconds before stopping. -1 indicates no timeout.
    void RunClawRollerEject(int timeout_s,bool slow);

    // Stop the claw roller
    void StopClawRollers();

    // Get whether there is currently a cube loaded in the claw
    bool IsCubeLoaded();

    // Set that there is a cube loaded in the claw
    void SetCubeLoaded();


    //==========================================================================
    // Arm Operation

    // Lift the arm to the 'up' position
    void LiftArm();

    // Drop the arm to the 'down' position
    void DropArm();

    // Get whether the arm is up
    //
    // This returns the state of the solenoid for the arm. Even though the arm
    // is fast it does take a finite time to move and this function does not
    // know when the movement is finished.
    bool IsArmUp();


    //==========================================================================
    // Lift Operation

    // Preset lift positions
    enum class LiftPosition {
    	kFloor,			// Lift at the lowest position
    	kArmLift,		// Lift is high enough to lift the arm without the cube catching on the robot frame
		kSwitch,		// Lift positioned to reach the switch
		kScale,			// Lift positioned to reach the scale
		kIntermediate	// Lift is between the preset positions, or not being driven to a position
    };

    // Drive the lift towards a given position
    //
    // position - Position to drive the lift towards
    void MoveLiftToPosition(LiftPosition position);

    // Get the current lift position
    LiftPosition GetLiftPosition();

    // Get the height of the lift carriage above the lowest position in inches
    double GetLiftHeightInch();

    // Get the height of a given preset list position in inches
    //
    // position - Preset position to convert to inches
    //
    // Returns the height in inches
    double GetLiftPositionHeightInch(LiftPosition position);

    // Get a scaling factor to apply to the drive base rotate and move speed due to
    // the lift being raised. Speed is reduced when the lift is high to increase
    // stability and prevent the robot tipping over.
    double GetLiftRaiseMovementScale();

    bool GetHighLiftSlowEnabled() { return m_high_lift_slow_enabled; }
    void SetHighLiftSlowEnabled(bool set) { m_high_lift_slow_enabled = set; }


private:
    //==========================================================================
	// Claw Rollers Implementation

    // Set the claw roller velocity
    //
    // velocity - velocity to run the rollers at (from -1 out to +1 in)
    void SetClawRollerVelocity(double velocity);

    // Start a timeout after which the roller motors will be stopped. They be be stopped
    // sooner, either explicitly, or because of high current indicating a cube has been grabbed.
    //
    // timeout_s - Time in seconds before stopping. -1 indicates no timeout.
    void StartRollerTimeout(int timeout_s);

    // Stop both the duration timeout and the timer for high current stopping
    void StopRollerTimeouts();

    // Check the roller current and stop the motors if high current is detected
    // indicating a cube has been grabbed.
    void CheckRollerCurrent();

    // Check if the timeout for the rollers has been exceeded and stop them if so.
    void CheckRollerTimeout();


    //==========================================================================
	// Lift Implementation

    // Drive the lift motors with the given percentage output
    //
    // percentage_velocity - Percentage output (from -1 full up to +1 full down)
	void DoLiftDrive(double percentage_velocity);

	// Drive the lift towards the desired position if one is set, otherwise
	// leave it as it is
	void DriveLiftToPosition();


    //==========================================================================
    // Member Variables

	TalonSRX* m_lift_speed_controller;				// The lift Talon SRX speed controller
	frc::Joystick* m_operator_joystick;			 	// The operator joystick, used to control the elevator

	frc::Solenoid* m_arm_solenoid;					// Solenoid for the arm. Up is the default position.
	frc::Solenoid* m_claw_solenoid;					// Solenoid for the claw. Close is the default position.

    TalonSRX* m_right_claw_speed_controller;		// Right Claw Talon SRX speed controller
    TalonSRX* m_left_claw_speed_controller;			// Left Claw Talon SRX speed controller
    frc::Timer m_claw_roller_timer;					// Timer for the claw roller

    RollerState m_roller_state;						// Current state of the claw rollers
    bool m_cube_loaded;								// Cube loaded flag
	bool m_roller_current_limit_exceeded;			// Whether the roller current limit has been exceeded
	int m_roller_timeout_s;							// Timeout in seconds after which to stop the rollers -1 means no timeout
	frc::Timer m_roller_timeout_timer;				// Timer for stopping the rollers after a timeout
	frc::Timer m_roller_current_timer;				// Timer for stopping the rollers after high current

	bool m_grab_trigger_pressed;					// Whether the grab trigger was previously pressed
	bool m_eject_trigger_pressed;					// Whether the eject trigger was previously pressed

	LiftPosition m_lift_position;					// The position the lift is currently driven to

    DriveElevator* m_elevator_floor_command;		// Command to drive the lift to the floor position
    DriveElevator* m_elevator_switch_command;		// Command to drive the lift to the switch position
    DriveElevator* m_elevator_scale_command;		// Command to drive the lift to the scale position
    DriveArm* m_arm_up_command;						// Command to lift the arm up (this includes a slightly lift of the lift)
    DriveArm* m_arm_down_command;					// Command to drop the arm down
    EjectClaw* m_eject_cube_command;				// Command to eject the cube

    bool m_high_lift_slow_enabled;

	int m_log_counter;								// Counter for logging only occasionally

	static const int kPidDefaultIdx = 0;
	static const int kTalonElevatorTimeoutMs = 10;
	static const int kRunProfileSlotIdx = 0;		// PID profile slot id to use when running the robot
	static const int kTuneProfileSlotIdx = 1;		// PID profile slot id to use when tuning the speed controller

	static constexpr double kLiftEncodeCountsPerInch = 500.0;	// Encoder counts to raise the lift carriage one inch
	static constexpr double kLiftFloorPositionInch = 0.0;		// Lift 'Floor' position in inches
	static constexpr double kLiftArmLiftPositionInch = 6.0;		// Lift 'ArmLift' position in inches
	static constexpr double kLiftSwitchPositionInch = 22.0;		// Lift 'Switch' position in inches
	static constexpr double kLiftScalePositionInch = 60.0;		// Lift 'Scale' position in inches
	static constexpr double kLiftPositionToleranceInch = 2.0; 	// Tolerance for determining if the lift is at a given position
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
