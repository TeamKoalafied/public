//==============================================================================
// Robot.cpp
//==============================================================================

#include "RobotConfiguration.h"

#include "Commands/AutonomousCommand.h"
#include "Commands/TestCommand.h"
#include "Subsystems/DriveBase.h"
#include "Subsystems/Pneumatics.h"
#include "Subsystems/TestSubsystem.h"
#include "Subsystems/VisionSystem.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/LidarSystem.h"
#include "PeriodicTimer.h"
#include "TemperatureMonitor.h"
#include "Subsystems/Leds.h"

#include <IterativeRobot.h>
#include <TimedRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <WPILib.h>

#include <stdio.h>



//==============================================================================
// Main Robot Class
//
// Because we are using a command based robot there is not much for this class to
// do. Its principal responsibilities are:
//  - Create the robot subsystems and initialise them
//  - Call the the scheduler in the periodic functions
//  - Setup the command for autonomous mode
//==============================================================================
//class Robot: public frc::IterativeRobot {
class Robot: public frc::TimedRobot {
public:
	//==========================================================================
	// Construction

	Robot() {
		m_drive_base = NULL;
		m_pneumatics = NULL;
		m_vision_system = NULL;
		m_elevator = NULL;
		m_lidar = NULL;

		m_autonomous_command = NULL;
		m_autonomous_started = false;

		m_test_subsystem = NULL;
	}

private:
	//==========================================================================
	// Robot Init/Periodic (from frc::IterativeRobotBase)

    void RobotInit() override {
        printf("==========================================================================\n");
        printf("RobotInit() starting...\n");

    	// Create and setup the robots subsystems
        m_drive_base = new DriveBase();
        m_elevator = new Elevator();
      	m_drive_base->Setup();

      	m_leds = new Leds();

    	m_pneumatics = new Pneumatics();
    	m_pneumatics->Setup();

// Disable the vision system because the camera is broken
//    	m_vision_system = new VisionSystem();
//    	m_vision_system->Setup();

    	m_elevator->Setup();

    	// Lidar MUST be connected before you enable this. It blocks waiting on low level communication.
//    	m_lidar = new LidarSystem();
//    	m_lidar->Setup();

    	// Create and setup the test subsystem. This is used for testing on a RoboRIO that is not
    	// part of the robot. This code should be always be commented out when checking into GitHub.
    	//m_test_subsystem = new TestSubsystem();
    	//m_test_subsystem->Setup();

        // Set up controls on the dashboard for choosing autonomous parameters
    	AutonomousCommand::SetupDashboard();
    	SmartDashboard::PutNumber("Rotate Gain", 0.006);

        printf("RobotInit() finished\n");
        printf("==========================================================================\n");
    }

    void RobotPeriodic() {
    	//m_temperature_monitor.PeriodicUpdate();
    	//SmartDashboard::PutNumber("Lift Temp", m_temperature_monitor.GetTemperature());

    	AutonomousCommand::EchoSettingsToDashboard();
    }


    //==========================================================================
    // Autonomous Init/Periodic (from frc::IterativeRobotBase)

	void AutonomousInit() override {
        printf("==========================================================================\n");
        printf("AutonomousInit()\n");

        Elevator::GetInstance().SetCubeLoaded();

        // Create the autonomous command. Then try to set it up (this gets information from the
        // dashboard and game state to configure itself for the correct actions). If setup is
        // successful then start the command. Setup can fail because the game data is not
        // available, which should not be possible according to WPI documentation
        // (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details),
        // but seems to happen in real games.
        delete m_autonomous_command;
        m_autonomous_command = new AutonomousCommand();
        m_autonomous_started = false;
        if (m_autonomous_command->SetupCommand()) {
        	m_autonomous_command->Start();
            m_autonomous_started = true;
        }
	}

	void AutonomousPeriodic() override {
		// If the autonomous command has not been started then try to set it up again
		if (!m_autonomous_started) {
	        if (m_autonomous_command->SetupCommand()) {
	        	m_autonomous_command->Start();
	        }
		}

        // Run the scheduler, which makes all the current commands run
        frc::Scheduler::GetInstance()->Run();
	}


    //==========================================================================
    // Teleop Init/Periodic (from frc::IterativeRobotBase)

	void TeleopInit() override {
        printf("==========================================================================\n");
        printf("TeleopInit()\n");
        //printf("COMMUNISM IS INEVITABLE\n");
        m_periodic_timer.Init();

        m_temperature_monitor.SetLedPattern((m_temperature_monitor.GetLedPattern() + 1)%10);

        // We do not cancel the autonomous command. If it is still running it is allowed to
        // continue. Either the driver or the operator can interrupt it using the joystick.
	}

	void TeleopPeriodic() override {
	    // Print out information about how often the periodic function is being called
	    m_periodic_timer.Periodic();

	    // Run the scheduler, which makes all the current commands run
	    frc::Scheduler::GetInstance()->Run();
	}


    //==========================================================================
    // Teleop Init/Periodic (from frc::IterativeRobotBase)

	void TestInit() override {
        printf("==========================================================================\n");
        printf("TestInit()\n");
 	}

	void TestPeriodic() override {
	}


    //==========================================================================
    // Disabled Init/Periodic (from frc::IterativeRobotBase)

	void DisabledInit() override {
        printf("==========================================================================\n");
        printf("DisabledInit()\n");

//    	Elevator::GetInstance().ClearState();
//        DriveBase::GetInstance().ClearState();
	}

    void DisabledPeriodic() override {
    }


	//==========================================================================
	// Member Variables

	DriveBase* m_drive_base;				// Drive base subsystem - speed controllers and joystick
	Pneumatics* m_pneumatics;				// Pneumatics subsystem - compressor
    VisionSystem* m_vision_system;			// Vision subsystem - camera & processing thread
    Elevator* m_elevator;					// Lift subsystem
    LidarSystem* m_lidar;					// Lidar subsystem
    Leds* m_leds;

    AutonomousCommand* m_autonomous_command;// Command to run in the autonomous period
	bool m_autonomous_started;				// Whether the autonomous command has been started

	PeriodicTimer m_periodic_timer;         // Timer for monitoring response times

	TestSubsystem* m_test_subsystem;		// Test subsystem

	TemperatureMonitor m_temperature_monitor;
};


//==============================================================================
// Robot Definition
//==============================================================================
START_ROBOT_CLASS(Robot)
