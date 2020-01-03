//==============================================================================
// TestFlashLed.h
//==============================================================================

#ifndef TestFlashLed_H
#define CycleLed_H

#include <Commands/Command.h>

// This command flashes one of the LEDs on for a given period of time
class TestFlashLed : public frc::Command {
public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // led_index - Index of the LED to flash
    // time_s - Time to flash the LED on for in seconds
	TestFlashLed(int led_index, double time_s);

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

	int m_led_index;            // Index of the LED to flash

};

#endif  // FlashLed_H
