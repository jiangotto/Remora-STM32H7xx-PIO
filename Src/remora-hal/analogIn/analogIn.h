#ifndef ANALOGIN_H
#define ANALOGIN_H

#include <cstdint>
#include <string>
#include "stm32h7xx_hal.h"
#include "../pinNames.h"
#include "../PinNamesTypes.h"
#include "../peripheralPins.h"
#include "../pin/pin.h"


class AnalogIn {
private:
    std::string portAndPin;
    uint8_t portIndex;
    uint16_t pinNumber;
    uint16_t pin;
    PinName pinName;
    
    ADC_HandleTypeDef handle;
    uint8_t channel;
    uint8_t differential;

    Pin* analogInPin;

    static uint32_t getADCChannelConstant(int channel);
    static void enableADCClock(ADC_TypeDef* adc);

public:
    AnalogIn(const std::string& portAndPin);
    uint16_t read();

};


#endif //ANALOGIN_H