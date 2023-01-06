
//==============================================================================
// TestFlashLed.h
//==============================================================================

#ifndef CycleLed_H
#define CycleLed_H

#include <frc/commands/Command.h>



// This command flashes one of the LEDs on for a given period of time
class CycleLed : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // led_index - Index of the LED to flash
    // time_s - Time to flash the LED on for in seconds
	CycleLed();

    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

private:
    //==========================================================================
	// Member Variables


};

#endif  // FlashLed_H
