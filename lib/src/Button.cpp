#include "Button.h"

// state changes. ~50ms of bouncing from high to low.
void Button::Update()
{
    bool bouncing = false; // prevents startTime from restarting while readings fluctuate
    static unsigned long startTime = 0;
    static int prevReading = digitalRead(pin);
    int reading = digitalRead(pin);
    

    if ((reading != prevReading) && !bouncing) {
        startTime = millis();
        bouncing = true;
        state = STATE_BUTTON_UNSTEADY;
    }

    prevReading = reading;

    if ((millis() - startTime) > BUTTON_PRESS_DEBOUNCE_DELAY) 
    {
        //if state changed (runs once per change)
        if (reading != state) 
        {
            bouncing = false;
            state = reading;
            if (state == STATE_BUTTON_UP) {
                pressTime = 0;
            }    
        }

        // If still pressing 
        if (state == STATE_BUTTON_PRESSED) {
            pressTime = millis() - startTime;
        }
    }
}