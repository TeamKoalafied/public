//==============================================================================
// Utilities.h
//==============================================================================

#ifndef SRC_UTILITIES_H_
#define SRC_UTILITIES_H_

#include <Joystick.h>


// Utilities functions for various helpful things.
// NOTE: This namespace was called 'Utilities' but that clashes with one of the
// CTRE Pheonix namespaces.
namespace KoalafiedUtilities {
    //==========================================================================
    // Joystick

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

};

#endif /* SRC_UTILITIES_H_ */
