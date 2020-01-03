//==============================================================================
// Robot.cpp
//==============================================================================

#include "RobotConfiguration.h"

#include "Subsystems/DriveBase.h"
#include "Subsystems/Manipulator.h"
#include "Subsystems/Pneumatics.h"
#include "Subsystems/VisionSystem.h"
#include "Commands/StartupCommand.h"
#include "PeriodicTimer.h"

#include <frc/TimedRobot.h>
#include <frc/WPILib.h>

#include <stdio.h>

using namespace frc;

//==============================================================================
// Main Robot Class
//
// Because we are using a command based robot there is not much for this class to
// do. Its principal responsibilities are:
//  - Create the robot subsystems and initialise them
//  - Call the the scheduler in the periodic functions
//  - Setup the command for autonomous mode
//==============================================================================
class Robot: public frc::TimedRobot {
public:
	//==========================================================================
	// Construction

	Robot() : m_periodic_timer("Periodic") {
		m_drive_base = NULL;
		m_manipulator = NULL;
		m_pneumatics = NULL;
		m_startup_command = NULL;
		m_vision_system = NULL;
	}

private:
	//==========================================================================
	// Robot Init/Periodic (from frc::IterativeRobotBase)

    void RobotInit() override {
        printf("==========================================================================\n");
        printf("RobotInit() starting...\n");

    	// Create and setup the robots subsystems
        m_drive_base = new DriveBase();
		m_manipulator = new Manipulator();
      	m_drive_base->Setup();

    	m_pneumatics = new Pneumatics();
    	m_pneumatics->Setup();

// Disable the vision system because the camera is broken
//    	m_vision_system = new VisionSystem();
//    	m_vision_system->Setup();

		m_manipulator->Setup();

        printf("RobotInit() finished\n");
        printf("==========================================================================\n");
    }

    void RobotPeriodic() {
    }


    //==========================================================================
    // Autonomous Init/Periodic (from frc::IterativeRobotBase)

	void AutonomousInit() override {
        printf("==========================================================================\n");
        printf("AutonomousInit()\n");

        delete m_startup_command;
        m_startup_command = new StartupCommand();
      	m_startup_command->Start();

        m_periodic_timer.Init();
	}

	void AutonomousPeriodic() override {
		// Time how long processing is taking
	    m_periodic_timer.PeriodicStart();

	    // Run the scheduler, which makes all the current commands run
	    frc::Scheduler::GetInstance()->Run();

	    // Print out information about how often the periodic function is being called and
		//  how long processing is taking
	    m_periodic_timer.PeriodicEnd();
	}


    //==========================================================================
    // Teleop Init/Periodic (from frc::IterativeRobotBase)

	void TeleopInit() override {
        printf("==========================================================================\n");
        printf("TeleopInit()\n");

        m_periodic_timer.Init();
	}

	void TeleopPeriodic() override {
		// Time how long processing is taking
	    m_periodic_timer.PeriodicStart();

	    // Run the scheduler, which makes all the current commands run
	    frc::Scheduler::GetInstance()->Run();

	    // Print out information about how often the periodic function is being called and
		//  how long processing is taking
	    m_periodic_timer.PeriodicEnd();
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
	}

    void DisabledPeriodic() override {
    }


	//==========================================================================
	// Member Variables

	DriveBase* m_drive_base;				// Drive base subsystem - speed controllers and joystick
	Manipulator* m_manipulator;				// Manipulator subsystem
	Pneumatics* m_pneumatics;				// Pneumatics subsystem - compressor

	StartupCommand* m_startup_command;		// Command that runs at the start of the game

    VisionSystem* m_vision_system;			// Vision subsystem - camera & processing thread
	PeriodicTimer m_periodic_timer;         // Timer for monitoring response times
};


//==============================================================================
// Robot Definition
//==============================================================================
int main() { return frc::StartRobot<Robot>(); }
