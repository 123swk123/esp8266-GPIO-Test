/*
	This user_main.c is part of esp8266 GPIO Demo and Testing.

  esp8266 GPIO Demo and Testing is free software: you can redistribute
  it and/or modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  esp8266 GPIO Demo and Testing is distributed in the hope that it will
  be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with esp8266 GPIO Demo and Testing.
  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ets_sys.h>
#include <osapi.h>
#include <user_interface.h>
#include <gpio.h>

#define GPIO_SIGMA_DELTA_ADDRESS            0x68
#define SIGMA_DELTA_ENABLE                      BIT16
#define SIGMA_DELTA_ENABLE_S                    16
#define SIGMA_DELTA_PRESCALAR                   0x000000ff
#define SIGMA_DELTA_PRESCALAR_S                 8
#define SIGMA_DELTA_TARGET                      0x000000ff
#define SIGMA_DELTA_TARGET_S                    0

#include "driver\uart.h"

#include "user_config.h"

typedef enum {
    DIVDED_BY_1 = 0,    //timer clock
    DIVDED_BY_16 = 4, //divided by 16
    DIVDED_BY_256 = 8,  //divided by 256
} TIMER_PREDIVED_MODE;

typedef enum {      //timer interrupt mode
    TM_LEVEL_INT = 1, // level interrupt
    TM_EDGE_INT   = 0,  //edge interrupt
} TIMER_INT_MODE;

typedef enum {
    FRC1_SOURCE = 0,
    NMI_SOURCE = 1,
} FRC1_TIMER_SOURCE_TYPE;

extern void ICACHE_FLASH_ATTR hw_timer_init(FRC1_TIMER_SOURCE_TYPE source_type, u8 req);
extern void  hw_timer_set_func(void (* user_hw_timer_cb_set)(void));
extern void  hw_timer_arm(u32 val);

#ifdef TEST_SOUND
extern const uint8_t u8arrSound1[41280];
#endif

os_timer_t myTimer;

void user_rf_pre_init(void)
{
}

void vTimer_cbk(void)
{
  static uint32_t i = 0;

#ifdef TEST_HW_TIMER_IO_SPEED
 	//Test open drain and normal output mode
	if(i == 0)
	  {
	    //normal mode
	    GPIO_REG_WRITE(GPIO_PIN_ADDR(4), GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE));
	    i = 1;
	  }
	else
	  {
	    //open drain mode
	    GPIO_REG_WRITE(GPIO_PIN_ADDR(4), GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE));
	    i = 0;
	  }
#endif

#ifdef TEST_SOUND
	//static ICACHE_RODATA_ATTR const uint8_t* u8ptrArr = u8arrSound1;

	//toggle gpio 12 to generated refernce signal
	if (GPIO_REG_READ(GPIO_OUT_ADDRESS) & 0x1000) {
	    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 0x1000);          //set gpio 12 = 0
  } else {
      GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 0x1000);          //set gpio 12 = 1
  }

	SET_PERI_REG_BITS(PERIPHS_GPIO_BASEADDR+GPIO_SIGMA_DELTA_ADDRESS, SIGMA_DELTA_TARGET, u8arrSound1[i], SIGMA_DELTA_TARGET_S);
	i++;
	if(i == sizeof(u8arrSound1))
    {
	    //u8ptrArr = u8arrSound1;
      i = 0;
    }
#endif

}

void ICACHE_FLASH_ATTR Sys_Init_Complete(void)
{
	os_printf("Sys_Init_Complete\r\n");
	os_printf("Starting Timer...\r\n");

//	os_timer_disarm(&myTimer);
//	os_timer_setfn(&myTimer, vOneTime, NULL);
//	os_timer_arm_us(&myTimer, 125, true);

	hw_timer_init(NMI_SOURCE, 1);
	//RTC_REG_WRITE(FRC1_CTRL_ADDRESS, 0xc2);
	hw_timer_set_func(vTimer_cbk);
	hw_timer_arm(125);

	os_printf("FRC1_CTRL_ADDRESS: %08lX\r\n", RTC_REG_READ(FRC1_CTRL_ADDRESS));
}

LOCAL void uart0_rx_intr_handler(void *para)
{
	uint8_t u8Temp;

	if((READ_PERI_REG(UART_INT_ST(0)) & (UART_RXFIFO_FULL_INT_ST | UART_RXFIFO_TOUT_INT_ST)))
	{
		os_printf("ISR:%lX\r\n",(READ_PERI_REG(UART_INT_ST(0))));

		uart_rx_intr_disable(0);
		//os_printf_plus("pRxBuffer->RcvBuffSize: %d\r\n", pRxBuffer->RcvBuffSize);

#if 0
		u8Temp = (READ_PERI_REG(UART_STATUS(0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;
		while(u8Temp--)
		{
			uart_tx_one_char(0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);
		}
#else
		u8Temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
		SET_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST);  /*clear all the received chars*/
		CLEAR_PERI_REG_MASK(UART_CONF0(0), UART_RXFIFO_RST);
#endif
		WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

		uart_rx_intr_enable(0);

		if(u8Temp == 'b') {system_restart();}
	}
}

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

void ICACHE_FLASH_ATTR _HW_Init(void)
{
  system_set_os_print(true);

  system_update_cpu_freq(SYS_CPU_160MHZ);

  uart_init(BIT_RATE_921600, BIT_RATE_921600);

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);  //set as gpio function

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);  //set as gpio function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);  //set as gpio function

  GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 0xF035);     //set gpio 0,2,4,5,12,13,14 & 15 as output

#ifdef TEST_HW_TIMER_IO_SPEED
  GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 0x10);          //set gpio 4 = 1 and in timer toggle the drive mode
#endif

#ifdef TEST_SOUND
  GPIO_REG_WRITE(GPIO_PIN_ADDR(4), GPIO_PIN_SOURCE_SET(SIGMA_AS_PIN_SOURCE));
  SET_PERI_REG_BITS(PERIPHS_GPIO_BASEADDR+GPIO_SIGMA_DELTA_ADDRESS, SIGMA_DELTA_ENABLE, 1, SIGMA_DELTA_ENABLE_S);
  SET_PERI_REG_BITS(PERIPHS_GPIO_BASEADDR+GPIO_SIGMA_DELTA_ADDRESS, SIGMA_DELTA_PRESCALAR, 20, SIGMA_DELTA_PRESCALAR_S);
#endif

  ETS_UART_INTR_ATTACH(uart0_rx_intr_handler,  NULL);
  uart_rx_intr_enable(0);

  system_timer_reinit();

  //disable wifi since we dont want them for io testing
  //this also reduces cpu load and power
  wifi_set_opmode_current(NULL_MODE);
  wifi_fpm_auto_sleep_set_in_null_mode(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);

  os_printf("HW_Init complete\r\n");

  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_GPIO0_U,READ_PERI_REG(PERIPHS_IO_MUX_GPIO0_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_GPIO2_U,READ_PERI_REG(PERIPHS_IO_MUX_GPIO2_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_GPIO4_U,READ_PERI_REG(PERIPHS_IO_MUX_GPIO4_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_GPIO5_U,READ_PERI_REG(PERIPHS_IO_MUX_GPIO5_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_MTDI_U,READ_PERI_REG(PERIPHS_IO_MUX_MTDI_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_MTCK_U,READ_PERI_REG(PERIPHS_IO_MUX_MTCK_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_MTMS_U,READ_PERI_REG(PERIPHS_IO_MUX_MTMS_U));
  os_printf("%08lX=%08lX\r\n", PERIPHS_IO_MUX_MTDO_U,READ_PERI_REG(PERIPHS_IO_MUX_MTDO_U));
}

void user_init(void)
{
	_HW_Init();
	system_init_done_cb(Sys_Init_Complete);
}
