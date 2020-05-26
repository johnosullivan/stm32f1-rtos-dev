/*
 *	TX:	A9  ====> RX of TTL serial
 *	RX:	A10 <==== TX of TTL serial (not used)
 *	CTS:	A11 (not used)
 *	RTS:	A12 (not used)
 *	Config:	8N1
 *	Baud:	38400
 *
 *
 *  Caution:
 *	Not all GPIO pins are 5V tolerant, so be careful to
 *	get the wiring correct.
 */

#include <FreeRTOS.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <task.h>
#include <queue.h>
#include <stdlib.h>
#include <math.h>

#define MAX 100

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>

using namespace std;

#include "clock.h"

//#include "i2c.h"
//#include "utils.h"

volatile uint32_t system_mirco = 0;

#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH                                             \
  0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH                                              \
  0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH                                              \
  0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP                                                     \
  0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP                                                      \
  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP                                                      \
  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D  /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041 /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2   /**< Soft Reset */
#define SHT31_HEATEREN 0x306D    /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066   /**< Heater Disable */

/*
static I2C_Control i2c;
static volatile bool readf = false;
static volatile int isr_count = 0;

static QueueHandle_t uart_txq;

static void uart_setup(void) {
  // UART
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	uart_txq = xQueueCreate(256,sizeof(char));
}*/

/*
static void i2c_setup(void) {
  // I2C
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO6|GPIO7);	// I2C
	gpio_set(GPIOB,GPIO6|GPIO7); // Idle high
	// AFIO_MAPR_I2C1_REMAP=0, PB6+PB7
	gpio_primary_remap(0,0);
	i2c_configure(&i2c,I2C1,1000);
}*/

static void gpio_setup(void) {
  // GPIO PC13
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(
    GPIOC,
    GPIO_MODE_OUTPUT_2_MHZ,
    GPIO_CNF_OUTPUT_PUSHPULL,
    GPIO13);
}

/*
static void uart_task(void *args __attribute__((unused))) {
	char ch;
	for (;;) {
		if ( xQueueReceive(uart_txq, &ch, 500) == pdPASS ) {
			while (!usart_get_flag(USART1,USART_SR_TXE)) {
					taskYIELD();
			}
			usart_send(USART1,ch);
		}
	}
}

static inline void uart_putc(char ch) {
	usart_send_blocking(USART1,ch);
}

static void uart_puts(const char *s) {
	for ( ; *s; ++s ) {
		xQueueSend(uart_txq,s,portMAX_DELAY);
	}
}

static void uart1_uint32_t(char * name, uint32_t value) {
	char bufValue[10];
	ltoa(value, bufValue, 10);
	uart_puts(name);
	uart_puts(": ");
	uart_puts(bufValue);
	uart_puts("\n\r");
}*/

static void demo_task1(void *args __attribute__((unused))) {
	for (;;) {
		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}


static void demo_task2(void *args __attribute__((unused))) {
	for (;;) {
		gpio_toggle(GPIOC,GPIO13);
		//uart1_uint32_t("Timer_Value", micro_second());
		micro_second_sleep(1000000);
	}
}

/*
static void i2c_status(void *args __attribute__((unused))) {
	for (;;) {
		I2C_Fails fc;
		fc = setjmp(i2c_exception);

		if (fc == I2C_Ok) {
			uart_puts("I2C_Ok");
		}
		if (fc == I2C_Addr_Timeout) {
			uart_puts("I2C_Addr_Timeout");
		}
		if (fc == I2C_Addr_NAK) {
			uart_puts("I2C_Addr_NAK");
		}
		if (fc == I2C_Write_Timeout) {
			uart_puts("I2C_Write_Timeout");
		}
		if (fc == I2C_Read_Timeout) {
			uart_puts("I2C_Read_Timeout");
		}
		uart_puts("\n\r");

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}*/

/*
static void sensor_i2c(void *args __attribute__((unused))) {
	// Configure I2C1
	for (;;) {
		uint16_t stat = 0xFF;
		i2c_start_addr(&i2c,SHT31_DEFAULT_ADDR,Write);
		i2c_write(&i2c,SHT31_READSTATUS >> 8);
		i2c_write(&i2c,SHT31_READSTATUS & 0xFF);
		i2c_stop(&i2c);

		i2c_start_addr(&i2c,SHT31_DEFAULT_ADDR,Read);
		stat = i2c_read(&i2c,true);
		i2c_stop(&i2c);

    stat <<= 8;

		i2c_start_addr(&i2c,SHT31_DEFAULT_ADDR,Read);
		stat |= i2c_read(&i2c,true);
		i2c_stop(&i2c);

		uart_puts("-------------------\n\r");

		if (stat != 0xFFFF) {
			uint8_t readbuffer[6];

			i2c_start_addr(&i2c,SHT31_DEFAULT_ADDR,Write);
			i2c_write(&i2c,SHT31_MEAS_HIGHREP >> 8);
			i2c_write(&i2c,SHT31_MEAS_HIGHREP & 0xFF);
			i2c_stop(&i2c);

			vTaskDelay(pdMS_TO_TICKS(20));

			i2c_start_addr(&i2c,SHT31_DEFAULT_ADDR,Read);
			for (size_t i = 0; i < sizeof(readbuffer); i++) {
    		readbuffer[i] = i2c_read(&i2c,true);
  		}
			i2c_stop(&i2c);

			int32_t stemp = (int32_t) (((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
		  stemp = ((4375 * stemp) >> 14) - 4500;
		  float temp = (float) stemp / 100.0f;

			char tempchar[5];
			ftoa(temp, tempchar, 2);
			uart_puts("Temp C: ");
			uart_puts(tempchar);
			uart_puts("\n\r");

      float fahrenheit = ((temp * 9)/5) + 32;
      char tempcharf[5];
			ftoa(fahrenheit, tempcharf, 2);
      uart_puts("Temp F: ");
			uart_puts(tempcharf);
			uart_puts("\n\r");

		  //uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
		  //float humidity = (shum * 100.0f) / 65535.0f;
		  //shum = (625 * shum) >> 12;
		  //float humidity = (float) shum / 100.0f;
			//uart1_uint32_t("humidity", humidity);
		}
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}*/

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

int main(void) {
	// CPU clock is 72 MHz
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

  // Clock Setup
  clock_setup();
  // I2C Setup
  // UART Setup
	//uart_setup();
  // GPIO Setup
  gpio_setup();
  // SPI Setup

	//xTaskCreate(uart_task,"UART",100,NULL,configMAX_PRIORITIES - 1,NULL);

	// Demoing the blinking light example using the delay from FreeRTOS (vTaskDelay->pdMS_TO_TICKS) and a self managed timer for the same results (micro_second_sleep)
  /*
  #if 1
  	xTaskCreate(demo_task1,"BLINK1",100,NULL,configMAX_PRIORITIES - 1,NULL);
  #else
  	xTaskCreate(demo_task2,"BLINK2",100,NULL,configMAX_PRIORITIES - 1,NULL);
  #endif
  */

  xTaskCreate(demo_task2, "BLINK1", 100, NULL, configMAX_PRIORITIES - 1, NULL);

	//xTaskCreate(sensor_i2c,"SHT31",100,NULL,configMAX_PRIORITIES - 1,NULL);
	//xTaskCreate(i2c_status,"I2CSTATUS",100,NULL,configMAX_PRIORITIES-1,NULL);

	vTaskStartScheduler();
	for (;;);
	return 0;
}
