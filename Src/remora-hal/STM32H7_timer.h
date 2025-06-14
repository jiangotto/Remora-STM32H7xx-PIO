#ifndef STM32H7_timer_H
#define STM32H7_timer_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <memory>

#include "../remora-core/thread/pruTimer.h"

#define TIM_PSC 1
#define APB1CLK SystemCoreClock/2
#define APB2CLK SystemCoreClock/2

class TimerInterrupt;
class pruThread;

class STM32H7_timer : public pruTimer {
	friend class timerInterrupt;

private:
    TIM_TypeDef* timer;
    IRQn_Type irq;
    int irqPriority;

public:
    STM32H7_timer(TIM_TypeDef* _timer, IRQn_Type _irq, uint32_t _frequency, pruThread* _ownerPtr, int _irqPriority = 0);
	
	void configTimer() override;
    void startTimer() override;
    void stopTimer() override;
    void timerTick() override;
};

#endif
