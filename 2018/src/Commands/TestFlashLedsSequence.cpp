//==============================================================================
// TestFlashLedsSequence.cpp
//==============================================================================

#include <Commands/TestFlashLed.h>
#include <Commands/TestFlashLedsSequence.h>


//==============================================================================
// Construction

TestFlashLedsSequence::TestFlashLedsSequence() :
    frc::CommandGroup("FlashLedsSequence") {
    // Flash the three LED one after the other for 1s each
    AddSequential(new TestFlashLed(0, 1.0));
    AddSequential(new TestFlashLed(1, 1.0));
    AddSequential(new TestFlashLed(2, 1.0));
}
