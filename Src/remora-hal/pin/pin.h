#ifndef PIN_H
#define PIN_H

#include <cstdint>
#include <string>
#include "stm32h7xx_hal.h"

#define INPUT 0x0
#define OUTPUT 0x1

#define NONE        0b000
#define OPENDRAIN   0b001
#define PULLUP      0b010
#define PULLDOWN    0b011
#define PULLNONE    0b100

class Pin {
private:
    std::string portAndPin;
    uint8_t mode;
    uint8_t modifier;
    uint8_t portIndex;
    uint16_t pinNumber;
    uint16_t pin;
    //uint32_t mode;
    uint32_t pull;
    GPIO_TypeDef* GPIOx;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    void configurePin();
    void initialisePin();
    void enableClock();

public:
    Pin(const std::string& portAndPin, int mode);
    Pin(const std::string& portAndPin, int mode, int modifier);
    bool get() const;
    void set(bool value);
    void setAsOutput();
    void setAsInput();
    void setPullNone();
    void setPullUp();
    void setPullDown();
};

#endif // PIN_H