//==============================================================================
// KoalafiedUtilities.h
//==============================================================================

#ifndef SRC_UTILITIES_H_
#define SRC_UTILITIES_H_

#include <frc/Joystick.h>
#include <ctre/Phoenix.h>



// Utilities functions for various helpful things.
// NOTE: This namespace was called 'Utilities' but that clashes with one of the
// CTRE Pheonix namespaces.
namespace KoalafiedUtilities {
    //==========================================================================
    // Joystick Utilities

	// Clear the 'pressed' and 'released' state of all buttons for a joystick.
	//
	// joystick - Joystick whose state is to be reset
	//
	// Note that this function is necessary because when querying the 'pressed' and
	// 'released' state for a button the answer is true if the event occurred since
	// the last time the function was called. If buttons are pressed during an
	// autonomous command they are stored and may cause spurious behaviour when control
	// with the joystick is resumed.
	void ClearJoystickButtonState(frc::Joystick* joystick);

    // Adjust a value by raising its absolute value to the given power, but preserving the sign
    //
    // value - number to adjust
    // power - power to raise the value to
    //
    // Returns a number with the same sign as 'value' but with the absolute value modified.
    double PowerAdjust(double value, double power);


    //==========================================================================
	// Talon SRX & Talon FX Utilities

	// Calculate a log F, the feed forward gain for a Talon SRX controller
	//
	// controller - Talon SRX controller to log the F gain for
	// scale - Scaling factor to convert encoder counts to 'speed' for logging. The
	//     meaning of 'speed' depends on the application.
	// name - Name for logging to distinguish when multiple controllers are used
	void CalculateAndLogF(TalonSRX* controller, double speed_scale, const char* name);


	// Calculate a log F, the feed forward gain for a Talon SRX controller
	//
	// controller - Talon SRX controller to log the F gain for
	// scale - Scaling factor to convert encoder counts to 'speed' for logging. The
	//     meaning of 'speed' depends on the application.
	// name - Name for logging to distinguish when multiple controllers are used
	void TuneDriveTalonSRX(TalonSRX* controller, const char* name, double drive, double max_rpm, bool close_loop);

    // Convert a velocity from RPM to native TalonSRX units for a Talon using a CTRE magnetic encoder
    //
    // velocity_rpm - velocity in RPM to convert
    //
    // Returns the velocity in TalonSRX units (encoder counts per update period)
    double TalonSRXCtreVelocityRpmToNative(double velocity_rpm);

    // Convert a velocity from native TalonSRX units to RPM for a Talon using a CTRE magnetic encoder
    //
    // velocity_native - velocity in TalonSRX units (encoder counts per update period) to convert
    //
    // Returns the velocity in RPM
    double TalonSRXCtreVelocityNativeToRpm(double velocity_native);

    // Convert a velocity from RPM to native TalonFX units
    //
    // velocity_rpm - velocity in RPM to convert
    //
    // Returns the velocity in TalonFX units (encoder counts per update period)
    double TalonFXVelocityRpmToNative(double velocity_rpm);

    // Convert a velocity from native TalonFX units to RPM
    //
    // velocity_native - velocity in TalonFX units (encoder counts per update period) to convert
    //
    // Returns the velocity in RPM
    double TalonFXVelocityNativeToRpm(double velocity_native);

}

#endif /* SRC_UTILITIES_H_ */
