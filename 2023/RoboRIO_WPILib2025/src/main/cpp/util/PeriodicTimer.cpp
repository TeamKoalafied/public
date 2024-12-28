//==============================================================================
// PeriodicTimer.cpp
//==============================================================================

#include "PeriodicTimer.h"


PeriodicTimer::PeriodicTimer(const char* name, bool enabled, Mode mode, int display_frequency,
                             units::second_t warning_max, units::second_t warning_ave) :
                             TimerBase(name, enabled, mode, display_frequency, warning_max, warning_ave) {
    m_first_period = true;

}


void PeriodicTimer::PeriodUpdate() {
    if (!m_first_period) {
        Update(m_timer.Get());
        m_first_period = false;
    }
    m_first_period = false;
    m_timer.Restart();
}
