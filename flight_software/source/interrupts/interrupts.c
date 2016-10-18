#include <interrupts.h>

#include <stdint.h>
#include <mission_timekeeper.h>

#include <pwm_input_hal.h>
#include <sf10_reader.h>
#include <px4flow_reader.h>
#include <simple_telemetry.h>


extern sf10_sensor_data_handler *h0;

/*
At the moment, libopencm3 appears not to have macros/functions
explicitly defined to globally disable/enable interrupts.
Thus, the following two wrapper functions for some assembler are
required to achieve this.

Taken from:
http://permalink.gmane.org/gmane.comp.lib.libopencm3/29
 */

/*
Globally ENABLE interrupts:
 */

inline void _enable_interrupts(void)
{
	asm volatile ("cpsie i");
}

/*
Globally DISABLE interrupts:
 */

inline void _disable_interrupts(void)
{
	asm volatile ("cpsid i");
}

/*
Set up 1 kHz Systick interrupt as system-wide time base.

Adapted from: 
https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/obldc/systick/systick.c
 */

void systick_setup(void)
{
	/*
		64 MHz SYSCLK /8 = 8 MHz Systick clock
	 */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/*
		Overflow at 8 MHz / (7999+1U) = 1 kHz:
	 */
	systick_set_reload(7999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

/*
Overrides the WEAK declaration of systick_tick_handler() in nvic.h:
 */

void sys_tick_handler(void)
{
	flag_scheduler_callback();
	update_mission_time_counter();
	rc_input_watchdog_callback();
}

void tim2_isr(void)
{
	timer2_isr_callback();
}

void tim3_isr(void)
{
	timer3_isr_callback();
}

void tim4_isr(void)
{
	timer4_isr_callback();
}

void tim8_cc_isr(void)
{
	timer8_isr_callback();
}

void tim1_brk_tim15_isr(void)
{
	timer15_isr_callback();
}

void usart1_exti25_isr(void)
{
	char c = usart_recv_blocking(USART1);
	sf10_reader_callback(h0, c);
}

void usart3_exti28_isr(void)
{
	char c = usart_recv_blocking(USART3);
	px4flow_interrupt_callback(c);
}


// connected to Odroid
void usart2_exti26_isr(void)
{
	char c;
	if(usart_get_interrupt_source(USART2, USART_ISR_RXNE))
	{
		c = (char)usart_recv_blocking(USART2);
		telem_uart_rx_callback(c);
	}
}

void uart4_exti34_isr(void)
{
	char c;
	if(usart_get_interrupt_source(UART4, USART_ISR_RXNE))
	{
		c = (char)usart_recv_blocking(UART4);
		// usart_send_blocking(UART4, c);
		BNO055_interrupt_handler(c);
	}
}
