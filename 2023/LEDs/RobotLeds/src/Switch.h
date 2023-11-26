//===============================================================================
// Switch.h
//===============================================================================

// Switch is a simple wrapper for debouncing a switch
class Switch
{
public:
    // Switch event
    enum Event
    {
        NONE,
        UP,
        DOWN
    };

    // Constructor
    //
    // gpio - GPIO number for the switch. Must be a valid digital input that
    //  supports an internal pullup.
    Switch(int gpio)
    {
        m_gpio = gpio;
        m_down = false;
        m_last_pin_state = LOW;
        m_last_transition_ms = 0;
        pinMode(m_gpio, INPUT_PULLUP);
    }

    // Sample the switch and update its state
    //
    // Returns if any event has occurred on the switch
    Event Update()
    {
        // Get the current instantaneous switch state
        int pin_state = digitalRead(m_gpio);

        // Check if there has been a transition since the last update and
        // if so restart the timer
        if (pin_state != m_last_pin_state) {
            m_last_transition_ms = millis();
        }
        m_last_pin_state = pin_state;
    
        // If the time since the last transistion exceeds the debounce delay
        // update the down state of the button, if necessary, and return
        // the appropriate event.
        if ((millis() - m_last_transition_ms) > ms_debounce_delay_ms) {
            bool down = pin_state == LOW;
            if (down != m_down) {
                m_down = down;
                return down ? DOWN : UP;
            }
        }

        // No event occurred on this update
        return NONE;
    }

    // Get whether the button is currently down
    bool IsDown() { return m_down; }

    // The debounce delay in milliseconds
    static const unsigned long ms_debounce_delay_ms = 50; 
    
private:
    int m_gpio;                         // GPIO number for the switch
    bool m_down;                        // Whether the switch is currently down
    int m_last_pin_state;               // The state of the switch at the last update
    unsigned long m_last_transition_ms; // THe time of the last transition of the switch state, measured by millis()
};