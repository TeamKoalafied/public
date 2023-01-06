//==============================================================================
// Leds.h
//==============================================================================

#ifndef Leds_H
#define Leds_H

#include "../TSingleton.h"
#include <frc/commands/Subsystem.h>
#include <frc/I2C.h>
#include <frc/Timer.h>



namespace frc {

}

// The led subsystem controls the leds.
class Leds : public TSingleton<Leds>, public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    Leds();

    // Destructor
    virtual ~Leds();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    //==========================================================================

    //Pattern Managing
    void SetLedPattern(int pattern);
    void IncrementPattern();

    //==========================================================================
    // Setup and Shutdown

    // Setup the Leds for operation
    void Setup();

    // Shutdown the Leds
    void Shutdown();

private:
    //==========================================================================
    // Member Variables
    frc::I2C m_arduino_i2c{frc::I2C::kOnboard, 8};
    static constexpr int kTotalPatterns = 5;
    int m_pattern = 1;
};

#endif  // Leds_H
