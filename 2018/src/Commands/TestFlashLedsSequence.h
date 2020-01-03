//==============================================================================
// TestFlashLedsSequence.h
//==============================================================================

#ifndef TestFlashLedsSequence_H
#define TestFlashLedsSequence_H

#include <Commands/CommandGroup.h>

 // This command flashed the three LED one after the other for 1s each
class TestFlashLedsSequence : public frc::CommandGroup {
public:
    //==========================================================================
    // Construction

    // Constructor
	TestFlashLedsSequence();
};

#endif  // FlashLedsSequence_H
