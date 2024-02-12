#ifndef BUTTON_H
#define BUTTON_H
#include <Arduino.h>

int BUTTON_PRESS_DEBOUNCE_DELAY = 50;

class Button
{
    int state = 0;
    unsigned long pressTime;

    uint8_t pin;
    uint8_t mode;
public:
    const int STATE_BUTTON_UNSTEADY = -1;
    const int STATE_BUTTON_UP = 0;
    const int STATE_BUTTON_DOWN = 1;
    const int STATE_BUTTON_PRESSED = 3;

    Button(uint8_t pin, uint8_t mode = INPUT_PULLUP)
    {
        PinMode(pin, mode);
    }
    
    // (Optional) Not required in setup. Use to change pinmode at runtime 
    void PinMode(uint8_t pin, uint8_t mode = INPUT_PULLUP)
    {
        this->pin = pin;
        this->mode = mode;
        pinMode(pin, mode);
    }

    void Update();
    int GetState() {return state;}
    int GetPressTime() {return pressTime;}
    void OnClick(void (*pointerFunction)(void))
    {
        pointerFunction();
    }
};
#endif