//==============================================================================
// Utilities.cpp
//==============================================================================

#include <Utilities.h>
#include <DriverStation.h>

//==============================================================================
// Construction


void KoalafiedUtilities::ClearJoystickButtonState(frc::Joystick* joystick)
{
	// Get the total number of buttons for the joystick. Note that the joystick state
	// comes from the DriverStation object.
	DriverStation& driver_station = DriverStation::GetInstance();
	int joystick_port = joystick->GetPort();
	int total_buttons = driver_station.GetStickButtonCount(joystick_port);

	// Query the 'pressed' and 'released' state for all buttons as this will reset them.
	for (int i = 1; i <= total_buttons; i++) {
		driver_station.GetStickButtonPressed(joystick_port, i);
		driver_station.GetStickButtonReleased(joystick_port, i);
	}
}


