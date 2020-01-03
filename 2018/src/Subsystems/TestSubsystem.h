//==============================================================================
// TestSubsystem.h
//==============================================================================

#ifndef SRC_SUBSYSTEMS_TESTSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_TESTSUBSYSTEM_H_

#include "TSingleton.h"
#include <DigitalOutput.h>
#include <Ultrasonic.h>
#include <Timer.h>
#include <PIDController.h>
#include <PIDOutput.h>
#include <PIDSource.h>
#include <Commands/Subsystem.h>
#include <SmartDashboard/SendableChooser.h>

namespace frc
{
class Joystick;
}


class TestSubsystem : public TSingleton<TestSubsystem>, public frc::Subsystem {
public:
    enum {
        kTotalLeds = 3      // Number of LEDs
    };

    //==========================================================================
    // Construction

    // Constructor
	TestSubsystem();

    // Destructor
    virtual ~TestSubsystem();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the test subsystem
    void Setup();

    // Shutdown the test subsystem
    void Shutdown();

    //==========================================================================
    // Operation

    // Get whether a given LED is current on
    //
    //  number - index of the LED (range 0 to kTotalLeds-1)
    bool GetLedOn(int number);

    // Turn and LED on or off
    //
    //  number - index of the LED (range 0 to kTotalLeds-1)
    //  on - whether to turn the LED on (true) or off (false)
    void SetLedOn(int number, bool on);


    void DoJoystick();

    //==========================================================================
    // Fake Driving

    void ResetDriving();

    void Drive(double drive_value);

    double GetPosition();


private:
    //==========================================================================
    //

    void UpdateFakeDriving();

    void StartPID();
    void StopPID();

    //==========================================================================
    //

    // Display the joystick position on the shuffle board
    void DisplayJoystickPosition();


	class MyPIDOutput: public frc::PIDOutput {
	public:
		MyPIDOutput(TestSubsystem& s);
		void PIDWrite(double output) override;
	private:
		TestSubsystem& m_s;
	};

	class MyPIDSource: public frc::PIDSource {
	public:
		MyPIDSource(TestSubsystem& s);
		virtual double PIDGet() override;
	private:
		TestSubsystem& m_s;
	};


	enum class Mode {
		kFloodTest,
		kNormal
	};

    //==========================================================================
    // Member Variables

	frc::Joystick* m_joystick = NULL;
	int m_log_flood_counter = 0;

    // The array of digital output objects for each of the LEDs
    frc::DigitalOutput* m_led_digital_outputs[kTotalLeds];

	Ultrasonic* m_ultrasonic;

	double m_position;
	double m_velocity;
	double m_acceleration;
	frc::Timer m_drive_timer;

	MyPIDOutput m_pid_output;
	MyPIDSource m_pid_source;
	frc::PIDController m_pid_controller { 1, 0.01, 10, m_pid_source, m_pid_output };

	frc::SendableChooser<Mode> m_mode_chooser;
								// A chooser for selecting the testing mode

};

#endif /* SRC_SUBSYSTEMS_TESTSUBSYSTEM_H_ */
