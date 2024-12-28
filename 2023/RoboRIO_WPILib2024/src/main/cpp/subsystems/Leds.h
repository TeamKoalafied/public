//==============================================================================
// Leds.h
//==============================================================================

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/I2C.h>
#include <frc/Timer.h>

namespace frc {

}


// The led subsystem controls the leds.
class Leds {
public:
    //==========================================================================
    // Publid Nested Types

    enum class Pattern {
        Off = 0,
        Cube = 3,
        CubeIntaking = 5,
        CubeGrabbed = 7,
        Cone = 4,
        ConeIntaking = 6,
        ConeGrabbed = 8
    };

    //==========================================================================
    // Construction

    // Constructor
    Leds();

    // Destructor
    ~Leds();

    //Pattern Managing
    void SetLedPattern(Pattern pattern);
    void IncrementPattern();

private:
    //==========================================================================
    // Member Variables

    frc::I2C m_arduino_i2c{frc::I2C::kMXP, 9};
    int m_pattern = 1;

    static constexpr int kTotalPatterns = 12;
};

