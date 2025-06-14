#ifndef PERIPHERALPINS_H
#define PERIPHERALPINS_H

#include "stm32h7xx_hal.h"
#include "pinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADC_1 = (int)ADC1_BASE,
    ADC_2 = (int)ADC2_BASE,
#if ADC3_BASE
    ADC_3 = (int)ADC3_BASE
#endif
} ADCName;

extern const PinMap PinMap_ADC[];

uint32_t pinmap_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_function(PinName pin, const PinMap *map);
uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_find_function(PinName pin, const PinMap *map);

#ifdef __cplusplus
}
#endif

#endif //PERIPHERALPINS_H
