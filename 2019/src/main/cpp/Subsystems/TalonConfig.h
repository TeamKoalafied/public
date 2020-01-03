#ifndef SRC_CONFIGS_H_
#define SRC_CONFIGS_H_

#include "RobotConfiguration.h"
#include "ctre/Phoenix.h"

struct masterConfigs {
    /*Hold all of the config data*/
	ctre::phoenix::motorcontrol::can::TalonSRXConfiguration _talon;

	void configs() {
		/*Construct all of the configurations with any set of values
		 *These are just arbitrary values to demonstrate the feature
         */

        //TalonSRX:
        _talon.peakOutputForward = +1.0f;
        _talon.peakOutputReverse = -1.0f;
        _talon.nominalOutputForward = RobotConfiguration::kDriveMotorNominalOutput;
        _talon.nominalOutputReverse = -RobotConfiguration::kDriveMotorNominalOutput;

        _talon.openloopRamp = RobotConfiguration::kDriveMotorRampRateS;
        _talon.closedloopRamp = RobotConfiguration::kDriveMotorRampRateS;

        _talon.peakCurrentLimit = RobotConfiguration::kDriveMotorPeakCurrentLimitA;
        _talon.peakCurrentDuration = RobotConfiguration::kDriveMotorPeakCurrentDurationMs;
        _talon.continuousCurrentLimit = RobotConfiguration::kDriveMotorContinuousCurrentLimitA;
        
        _talon.slot0.kP = RobotConfiguration::kDriveBasePidP;
        _talon.slot0.kI = RobotConfiguration::kDriveBasePidI;
        _talon.slot0.kD = RobotConfiguration::kDriveBasePidD;
        _talon.slot0.kF = RobotConfiguration::kDriveBasePidF;
    }
};

struct slaveConfigs {
    /*Hold all of the config data*/
	ctre::phoenix::motorcontrol::can::TalonSRXConfiguration _talon;

	void configs() {
		/*Construct all of the configurations with any set of values
		 *These are just arbitrary values to demonstrate the feature
         */

        //TalonSRX:
        _talon.openloopRamp = RobotConfiguration::kDriveMotorRampRateS;
        _talon.closedloopRamp = RobotConfiguration::kDriveMotorRampRateS;

        _talon.peakCurrentLimit = RobotConfiguration::kDriveMotorPeakCurrentLimitA;
        _talon.peakCurrentDuration = RobotConfiguration::kDriveMotorPeakCurrentDurationMs;
        _talon.continuousCurrentLimit = RobotConfiguration::kDriveMotorContinuousCurrentLimitA;
    }
};
    
#endif /* SRC_CONFIGS_H_ */