#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "clock.h"

volatile uint32_t system_mirco = 0;

void micro_second_sleep(uint32_t delay)
{
	uint32_t wake = system_mirco + delay;
	while (wake > system_mirco);
}

uint32_t micro_second(void)
{
	return system_mirco;
}

void tim2_isr(void)
{
	system_mirco++;
	TIM_SR(TIM2) &= ~TIM_SR_UIF;
}

void clock_setup(void)
{
  // TIM2 Setup / micro_second / us
	rcc_periph_clock_enable(RCC_TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);

	volatile uint32_t freq = 72;

	TIM_CNT(TIM2) = 1;
	TIM_PSC(TIM2) = freq;
	TIM_ARR(TIM2) = 1;
	TIM_DIER(TIM2) |= TIM_DIER_UIE;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;
}
