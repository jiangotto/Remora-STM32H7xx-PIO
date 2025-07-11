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

typedef enum {
    SPI_1 = (int)SPI1_BASE,
    SPI_2 = (int)SPI2_BASE,
    SPI_3 = (int)SPI3_BASE,
    SPI_4 = (int)SPI4_BASE,
    SPI_5 = (int)SPI5_BASE,
    SPI_6 = (int)SPI6_BASE
} SPIName;

extern const PinMap PinMap_ADC[];
extern const PinMap PinMap_SPI_MOSI[];
extern const PinMap PinMap_SPI_MISO[];
extern const PinMap PinMap_SPI_SCLK[];
extern const PinMap PinMap_SPI_SSEL[];

uint32_t pinmap_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_function(PinName pin, const PinMap *map);
uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_find_function(PinName pin, const PinMap *map);
uint32_t pinmap_merge(uint32_t a, uint32_t b);
PinName portAndPinToPinName(const char* portAndPin);

#ifdef __cplusplus
}
#endif

#endif //PERIPHERALPINS_H
