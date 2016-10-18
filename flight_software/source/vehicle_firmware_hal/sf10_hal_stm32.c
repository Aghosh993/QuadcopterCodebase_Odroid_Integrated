/*
 * sf10_hal.c
 *
 *  Created on: Jul 3, 2015
 *      Author: aghosh01
 */

#include "sf10_hal_stm32.h"

#define USART1_USE_RX_ISR   1

static void usart1_setup(void)
{
    /* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    #ifdef USART1_USE_RX_ISR
        // Unmask receive interrupt
        usart_enable_rx_interrupt(USART1);
        // Make sure the interrupt is routed through the NVIC
        nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);
    #endif
    /* Finally enable the USART. */
    usart_enable(USART1);
}

void send_byte_to_sf10A(uint8_t send_byte)
{
    usart_send_blocking(USART1, send_byte);
}

void sf10_hal_setup_stm32_serial(void)
{
    usart1_setup();
}
