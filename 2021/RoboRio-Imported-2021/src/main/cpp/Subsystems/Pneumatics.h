//==============================================================================
// Pneumatics.h
//==============================================================================

#ifndef Pneumatics_H
#define Pneumatics_H

#include "../TSingleton.h"
#include <frc/commands/Subsystem.h>



namespace frc {
    class AnalogInput;
    class Compressor;
}

// The pneumatics subsystem controls the compressor.
//
// TODO Do we need a WaitForPressure command like the PacGoat example?
// TODO Can we get the current pressure? Or only get if the pressure switch is 'low'.
class Pneumatics : public TSingleton<Pneumatics>, public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    Pneumatics();

    // Destructor
    virtual ~Pneumatics();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void InitDefaultCommand() override;
    virtual void Periodic() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the pneumatics for operation
    void Setup();

    // Shutdown the pneumatics
    void Shutdown();

private:
    //==========================================================================
    // Member Variables

    frc::Compressor* m_compressor;      // The compressor object

    // Analog input for measuring the air pressure
    frc::AnalogInput* m_pressure_guage_input;
};

#endif  // Pneumatics_H
