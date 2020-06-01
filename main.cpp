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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>

using namespace std;

#include "clock.hpp"
#include "aes/aes.hpp"
//#include "i2c.h"
//#include "utils.h"

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


//static I2C_Control i2c;
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
}

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

/*
static void uart1_uint32_t(char * name, uint32_t value) {
	char bufValue[10];
	ltoa(value, bufValue, 10);
	uart_puts(name);
	uart_puts(": ");
	uart_puts(bufValue);
	uart_puts("\n\r");
}
*/

static void demo_task1(void *args __attribute__((unused))) {
	for (;;) {
		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(1000));
    uart_puts("#{}\n\r");
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

static void phex(uint8_t* str)
{
    uint8_t len = 16;
    unsigned char i;
    for (i = 0; i < len; ++i)
        uart_puts(str[i]);
    uart_puts("\n");
}

int main(void) {
	// CPU clock is 72 MHz
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

  // Clock Setup
  clock_setup();
  // I2C Setup
  //i2c_setup();
  // UART Setup
	uart_setup();
  // GPIO Setup
  gpio_setup();
  // SPI Setup

	xTaskCreate(uart_task,"UART",100,NULL,configMAX_PRIORITIES - 1,NULL);

	// Demoing the blinking light example using the delay from FreeRTOS (vTaskDelay->pdMS_TO_TICKS) and a self managed timer for the same results (micro_second_sleep)
  /*
  #if 1
  	xTaskCreate(demo_task1,"BLINK1",100,NULL,configMAX_PRIORITIES - 1,NULL);
  #else
  	xTaskCreate(demo_task2,"BLINK2",100,NULL,configMAX_PRIORITIES - 1,NULL);
  #endif
  */

  xTaskCreate(demo_task1, "BLINK1", 100, NULL, configMAX_PRIORITIES - 1, NULL);

  // AES 256 Example
   uint8_t i;

   // 128bit key
   uint8_t key[16] =        { (uint8_t) 0x2b, (uint8_t) 0x7e, (uint8_t) 0x15, (uint8_t) 0x16, (uint8_t) 0x28, (uint8_t) 0xae, (uint8_t) 0xd2, (uint8_t) 0xa6, (uint8_t) 0xab, (uint8_t) 0xf7, (uint8_t) 0x15, (uint8_t) 0x88, (uint8_t) 0x09, (uint8_t) 0xcf, (uint8_t) 0x4f, (uint8_t) 0x3c };
   // 512bit text
   uint8_t plain_text[64] = { (uint8_t) 0x6b, (uint8_t) 0xc1, (uint8_t) 0xbe, (uint8_t) 0xe2, (uint8_t) 0x2e, (uint8_t) 0x40, (uint8_t) 0x9f, (uint8_t) 0x96, (uint8_t) 0xe9, (uint8_t) 0x3d, (uint8_t) 0x7e, (uint8_t) 0x11, (uint8_t) 0x73, (uint8_t) 0x93, (uint8_t) 0x17, (uint8_t) 0x2a,
                              (uint8_t) 0xae, (uint8_t) 0x2d, (uint8_t) 0x8a, (uint8_t) 0x57, (uint8_t) 0x1e, (uint8_t) 0x03, (uint8_t) 0xac, (uint8_t) 0x9c, (uint8_t) 0x9e, (uint8_t) 0xb7, (uint8_t) 0x6f, (uint8_t) 0xac, (uint8_t) 0x45, (uint8_t) 0xaf, (uint8_t) 0x8e, (uint8_t) 0x51,
                              (uint8_t) 0x30, (uint8_t) 0xc8, (uint8_t) 0x1c, (uint8_t) 0x46, (uint8_t) 0xa3, (uint8_t) 0x5c, (uint8_t) 0xe4, (uint8_t) 0x11, (uint8_t) 0xe5, (uint8_t) 0xfb, (uint8_t) 0xc1, (uint8_t) 0x19, (uint8_t) 0x1a, (uint8_t) 0x0a, (uint8_t) 0x52, (uint8_t) 0xef,
                              (uint8_t) 0xf6, (uint8_t) 0x9f, (uint8_t) 0x24, (uint8_t) 0x45, (uint8_t) 0xdf, (uint8_t) 0x4f, (uint8_t) 0x9b, (uint8_t) 0x17, (uint8_t) 0xad, (uint8_t) 0x2b, (uint8_t) 0x41, (uint8_t) 0x7b, (uint8_t) 0xe6, (uint8_t) 0x6c, (uint8_t) 0x37, (uint8_t) 0x10 };

   uart_puts("Plain-Text:\n");
   uart_puts("------");
   for (i = (uint8_t) 0; i < (uint8_t) 4; ++i)
   {
       phex(plain_text + i * (uint8_t) 16);
   }
   uart_puts("\n");

   /*
   uart_puts("key:\n");
   phex(key);
   uart_puts("\n");

   // print the resulting cipher as 4 x 16 byte strings
   uart_puts("ciphertext:\n");

   struct AES_ctx ctx;
   AES_init_ctx(&ctx, key);

   for (i = 0; i < 4; ++i)
   {
     AES_ECB_encrypt(&ctx, plain_text + (i * 16));
     phex(plain_text + (i * 16));
   }
   uart_puts("\n");
   */

	vTaskStartScheduler();
	for (;;);
	return 0;
}
