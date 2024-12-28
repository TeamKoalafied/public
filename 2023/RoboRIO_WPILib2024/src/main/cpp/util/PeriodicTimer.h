//==============================================================================
// PeriodicTimer.h
//==============================================================================

#ifndef SRC_PERIODICTIMER_H_
#define SRC_PERIODICTIMER_H_

#include <frc/Timer.h>
#include <string>
#include <units/time.h>

using namespace frc;

namespace frc { class ShuffleboardTab; }
namespace frc { class SimpleWidget; }


// A timer for monitoring how long some process takes. The PeriodicStart() is called before
// the processing and the PeriodicEnd() function after. The timer tracks both the time taken
// for processing and the time between the start of each lot of processing. Tracking
// the time between is useful for when this class is use to track execution of one of
// the robot periodic functions. 
class PeriodicTimer {
public:
    //
    enum Mode {
        Manual                  = 0,  // Update only occur when DoDisplay() is called manually
        AutoPeriods             = 1,  // Updates occur automaticall after a set number of periods
        AutoMilliseconds        = 2,  // Updates occur automaticall after a set millisecond duration
        UpdateMask              = 3,  // Mask for extracting when updates occur
        ShowPeriodicWarnings    = 4,  // Display warnings about periodic overruns
        ShowPeriodicStats       = 8,  // Display statistics for periodic timing
        ShowPeriodic            = ShowPeriodicWarnings | ShowPeriodicStats,
        ShowProcessingWarnings  = 16, // Display warnings about processing overruns
        ShowProcessingStats     = 32, // Display statistics for processing timing
        ShowProcessing          = ShowProcessingWarnings | ShowProcessingStats,
        ShowWarnings            = ShowPeriodicWarnings | ShowProcessingWarnings,
        ShowStats               = ShowPeriodicStats |ShowProcessingStats,
        ShowAll                 = ShowWarnings | ShowStats,
        DefaultMode             = AutoPeriods | ShowAll
    };

    //==========================================================================
    // Construction

    // Constructor
    //
    // name - Name of the timer. Shown in warnings and statistics.
    // mode - Mode flags that control display
    // display_frequency - Frequency or automatic updates (either periods or milliseconds
    //      depending on the mode).  
    PeriodicTimer(const char* name, Mode mode = DefaultMode, int display_frequency = 100, bool enabled = true);

    // Destructor
    ~PeriodicTimer();

    //==========================================================================
    // Operation

    // Initialises the timer. Call this function from the frc::IterativeRobot *Init() functions.
    void Init();

    // Updates the periodic pocessing timer. Call this function at the start of frc::IterativeRobot *Periodic() functions
    void PeriodicStart();

    // Updates the timer and log results at intervals. Call this function at the end of frc::IterativeRobot *Periodic() functions
    void PeriodicEnd();

    // Set the timer to automatically display warning/statistics after a regular delay
    //
    // mode - Mode flags that control display
    // display_frequency - Frequency or automatic updates (either periods or milliseconds
    //      depending on the mode).  
    void SetMode(Mode mode, int display_frequency);

    // Set the time limits for when warnings will be shown for processing timing
    //
    // warning_processing_max - Time above which to warn about maximum processing
    // warning_processing_ave - Time above which to warn about average processing
    void SetProcessingLimits(units::second_t warning_processing_max,
                             units::second_t warning_processing_ave);

    // Set the time limits for when warnings will be shown for periodic timing
    //
    // warning_periodic_max - Time above which to warn about maximum periodic
    // warning_periodic_ave - Time above which to warn about average periodic
    void SetPeriodicLimits(units::second_t warning_periodic_max,
                           units::second_t warning_periodic_ave);

    // Display warning and/or statistics and reset the statistic
    //
    // mode - Mode flags that control display
    void DoManualDisplay(Mode mode);

    // Get whether the timers is enabled
    bool GetEnabled() const {
        return m_enabled;
    }

    // Get whether the timers is enabled
    void SetEnabled(bool enabled) {
        m_enabled = enabled;
        Init();
    }



private:
    //==========================================================================
    // Private Nested Types

    // Struct containing widgets for a single timer position
    struct TimerWidgets {   
        frc::SimpleWidget* m_current_graph_widget;
        frc::SimpleWidget* m_current_widget;
        frc::SimpleWidget* m_average_widget;
        frc::SimpleWidget* m_max_widget;
        frc::SimpleWidget* m_min_widget;
    };



    //==========================================================================
    // Logging

    void ResetStatistics();

    // Log the raw deltas to the console
 //   void LogAllDeltas();

    // Log summary information to the console (Min, Max, Average)
    //
    // mode - Mode flags that control display
    void LogSummary(Mode mode);


    //==========================================================================
    // Shuffleboard

    // Setup any shuffle board widgets required
    void SetupShuffleboard();


    TimerWidgets* SetupTimerWidgets(frc::ShuffleboardTab& shuffleboard_tab, std::string type);


    void UpdateShuffleboard(units::millisecond_t processing, units::millisecond_t periodic);


    void UpdateTimerWidgets(TimerWidgets* timer_widgets, units::millisecond_t current, units::millisecond_t average,
                            units::millisecond_t min, units::millisecond_t max);


    //==========================================================================
    // Member Variables

    std::string m_name;                     // Name of the timer
    Mode m_mode;                            // Mode flags that control display
    int m_display_frequency;                // Frequency or automatic updates (either periods
                                            //  or milliseconds depending on the mode).
    bool m_enabled;                         // Whether the timer is enabled


    frc::Timer m_period_timer;              // A timer that measures elapsed time, between periods
    frc::Timer m_processing_timer;          // A timer that measures processing time for one periods
    bool m_first_period;                    // Flag to indicate that the very first period is being measured 

    units::millisecond_t m_total_processing;     // Total processing time recorded (seconds)
    units::millisecond_t m_min_processing;       // Minimum processing time recorded (seconds)
    units::millisecond_t m_max_processing;       // Maximum processing time recorded (seconds)
    units::millisecond_t m_total_periodic;       // Total periodic time recorded (seconds)
    units::millisecond_t m_min_periodic;         // Minimum periodic time recorded (seconds)
    units::millisecond_t m_max_periodic;         // Maximum periodic time recorded (seconds)
    int m_total_periods;                    // Number of processing periods that have occurred

    units::millisecond_t m_warning_processing_max;   // Time above which to warn about maximum processing
    units::millisecond_t m_warning_processing_ave;   // Time above which to warn about average processing
    units::millisecond_t m_warning_periodic_max; // Time above which to warn about maximum periodic
    units::millisecond_t m_warning_periodic_ave; // Time above which to warn about average periodic

    TimerWidgets* m_periodic_widgets;       // Widgets for periodic timing. nullptr if not used.
    TimerWidgets* m_processing_widgets;     // Widgets for processing timing. nullptr if not used.

    static int ms_widget_count;

 //   static const int TOTAL_DELTAS = 100;    // Number of deltas accumulated before logging
 //   double m_period_deltas[TOTAL_DELTAS];   // List of recorded deltas (seconds)
 //   double m_processing_times[TOTAL_DELTAS];// List of recorded processing times (seconds)
 //   int m_delta_index;                      // Index of the next delta to record or -1 before the first period
};

#endif /* SRC_PERIODICTIMER_H_ */
