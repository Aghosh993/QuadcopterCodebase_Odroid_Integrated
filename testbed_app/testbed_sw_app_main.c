/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
	C Standard Library/Newlib includes:
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
	HAL/HW-specific includes:
 */

#include <hal_common_includes.h>

/*
	Avionics software-specific includes:
 */

#include <interrupts.h>
#include <mission_timekeeper.h>
#include <imu.h>
#include <pwm_input.h>
#include <QuadRotor_PWM.h>
#include <comp_filter.h>
#include <pid_controller.h>
#include <lidar_lite_v1.h>
#include <quadrotor_comm.h>

/*
	Shamelessly stolen from I2C example in libopencm3-examples,
	but sharing is caring, right? Right? Okay.
 */

// #define LRED GPIOE, GPIO9
// #define LORANGE GPIOE, GPIO10
// #define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
// #define LRED2 GPIOE, GPIO13
// #define LORANGE2 GPIOE, GPIO14
// #define LGREEN2 GPIOE, GPIO15

/*
	Approx active LOC (Lines of code: ~3200)
 */

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA2,3 			-> 			USART2
	PE8,9,10,11,12,13,14,15 -> 	LEDs
	PB6,7 			-> 			I2C
	PA5,6,7 		-> 			SPI1 MISO,MOSI,CLK
	PE3 			-> 			SPI1 CS (user-controlled) for L3GD20
	PD3 			-> 			Timer2 Input capture
	PC6 			-> 			Timer3 Input capture
	PD12 			-> 			Timer4 Input capture
	PA15 			->			Timer8 Input capture
	PA8,9,10,PE14 	-> 			Timer1 PWM Output
 */

/*
 A basic routine to adjust system clock settings to get SYSCLK
 to 64 MHz, and have AHB buses at 64 MHz, and APB Bus at 32 MHz (its max speed)
 Copied from:
 https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f3/stm32f3-discovery/adc/adc.c
 */

/*
	External oscillator required to clock PLL at 72 MHz:
 */
static void set_system_clock(void)
{
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
}

static void led_gpio_setup(void)
{
	/* Enable GPIOE clock. */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Set GPIO8 and 12 (in GPIO port E) to 'output push-pull'. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void usart2_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

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

	// Unmask receive interrupt
	// usart_enable_rx_interrupt(USART1);
	// Make sure the interrupt is routed through the NVIC
	// nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

#ifdef INCLUDE_PWM_TEST_SHELL

static void pwm_test_shell(void)
{
	float pwm_setval = 0.0f;
	char cmd;
	uint8_t err = 0U;
	// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MAX_VAL+100);
	// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MAX_VAL+100);
	while(1)
	{
		scanf("%c", &cmd);
		switch(cmd)
		{
			case 'p':
			err=0U;
			pwm_setval += 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'n':
			err=0U;
			pwm_setval -= 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'o':
			err=0U;
			pwm_setval += 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'b':
			err=0U;
			pwm_setval -= 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'a':
			err=0U;
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			break;
			case 's':
			err=0U;
			QuadRotor_motor1_stop();
			QuadRotor_motor2_stop();
			QuadRotor_motor3_stop();
			QuadRotor_motor4_stop();
			break;
			case 'z':
			err=0U;
			pwm_setval = 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			default:
			err=1;
			break;
			// case 'A':
			// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MIN_VAL-100);
			// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MIN_VAL-100);
			// err=0;
			// break;
			// case 'B':
			// timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MAX_VAL+100);
			// timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MAX_VAL+100);
			// err=0;
			// break;
		}
		if(!err)
		{
			printf("%c %f\r\n", cmd, pwm_setval);
		}
		else
		{
			printf("Usage: [n,p]: Decrease/Increase PWM fine precision\r\n");
			printf("[b,o]: Decrease/Increase PWM course precision\r\n");
			printf("[a] to start output, [s] to [s]top output\r\n");
			printf("[z] to set approx 1.07 ms pulse output and keep output enabled\r\n");
		}
	}
}

#endif

static volatile uint8_t angular_control_update_flag;
static volatile uint8_t height_control_update_flag;

static void vehicle_gnc_update(imu_scaled_data_struct *imu_data, filtered_quadrotor_state *st_vector, rc_joystick_data_struct *user_joy_input)
{
	gpio_toggle(LBLUE2);
	float roll_cmd, pitch_cmd, yaw_cmd;
	float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;

	float user_height_command;

	double motor_commands[4];

	// gpio_toggle(LBLUE2);
	/*
		Read Gyro, accelerometer and magnetometer sensors,
		and scale data to engineering (i.e. SI) units.

		Takes about 510 Microseconds to run, on 64 MHz system main clock,
		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 64.

		Takes about 488 Microseconds to run, on 64 MHz system main clock,
		and using 400 KHz I2C bus clock along with SPI Clock Divisor of 32.
	 */
	get_scaled_imu_data(imu_data);

	/*
		Obtain filtered vehicle state through complementary filter.
		Takes about 25 Microseconds to run, on 64 MHz system main clock
	 */
	get_filtered_vehicle_state(st_vector, imu_data);

	if(user_joy_input->vertical_channel_validity == CHANNEL_INVALID || user_joy_input->roll_channel_validity == CHANNEL_INVALID ||
		user_joy_input->pitch_channel_validity == CHANNEL_INVALID || user_joy_input->yaw_channel_validity == CHANNEL_INVALID)
	{
		disable_controller();
		QuadRotor_motor1_stop();
		QuadRotor_motor2_stop();
		QuadRotor_motor3_stop();
		QuadRotor_motor4_stop();
	}

	/*
		Scale joystick vertical axis to user openloop vertical command:
	 */
	user_height_command = ((user_joy_input->vertical_channel_value)*0.5f)+0.5f;

	roll_cmd = 1.0f*user_joy_input->roll_channel_value;
	pitch_cmd = -1.0f*user_joy_input->pitch_channel_value;
	yaw_cmd = -1.0f*user_joy_input->yaw_channel_value;

	if(get_flag_state(angular_control_update_flag) == STATE_PENDING)
	{
		reset_flag(angular_control_update_flag);
		// generate_rate_commands(st_vector, roll_cmd, pitch_cmd, yaw_cmd, &roll_rate_cmd, &pitch_rate_cmd, &yaw_rate_cmd);
		// gpio_toggle(LBLUE2);
	}

	// rate_controller_update(motor_commands, imu_data, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, user_height_command);
	
	// QuadRotor_set_all_motors(motor_commands);
}

static float get_altitude_offset(lidar_lite_sensor_instance* ll_height_sensor_instance)
{
	uint8_t i = 0U;
	float running_sum = 0.0f;
	timekeeper_delay(35U);
	for(i = 0U; i < 100; ++i)
	{
		lidar_lite_trigger_measurement();
		timekeeper_delay(35U);
		running_sum += (float)lidar_lite_get_raw_data()/(float)100;
	}
	lidar_lite_initialize_instance(ll_height_sensor_instance);
	return running_sum/(float)100;
}

float kalman_roll = 0.0f;

void kalman_estimate(imu_scaled_data_struct *imu_data, filtered_quadrotor_state *st)
{

}

void aux_imu_i2c_bus_setup(void)
{
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_set_i2c_clock_hsi(I2C2);

	i2c_reset(I2C2);

	gpio_mode_setup(I2C2_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C2_SCL_PIN | I2C2_SDA_PIN);
	gpio_set_af(I2C2_GPIO, GPIO_AF4, I2C2_SCL_PIN | I2C2_SDA_PIN);

	i2c_peripheral_disable(I2C2);

	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C2);
	i2c_set_digital_filter(I2C2, I2C_CR1_DNF_DISABLED);

	//Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
	// in TIMINGR

	// i2c_100khz_i2cclk8mhz(I2C2);
	i2c_set_prescaler(I2C2,1);
	i2c_set_scl_low_period(I2C2, 0x02);
	i2c_set_scl_high_period(I2C2, 0x03);
	i2c_set_data_hold_time(I2C2, 0x01);
	i2c_set_data_setup_time(I2C2, 0x03);

	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(I2C2);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C2);
	i2c_peripheral_enable(I2C2);
}

uint8_t aux_imu_read_reg(uint8_t reg)
{
	uint8_t retval;
	read_i2c(I2C2, 0x28, reg, 1, &retval);
	return retval;
}

void get_aux_imu_quat(uint8_t *ret)
{
	ret[0] = aux_imu_read_reg(0x16);
	ret[1] = aux_imu_read_reg(0x17);
}

static int aux_imu_usart_read_single_reg(uint8_t reg)
{
	usart_send_blocking(USART1, 0xAA);
	usart_send_blocking(USART1, 0x01);
	usart_send_blocking(USART1, reg);
	usart_send_blocking(USART1, 0x01);

	int res = usart_recv_blocking(USART1);
	int len = 0;
	int ret = 0;

	if(res == 0xEE)
	{
		return -1;//usart_recv_blocking(USART1);
	}
	else
	{
		len = usart_recv_blocking(USART1);
		ret = usart_recv_blocking(USART1);
		return ret;
	}
}

static int aux_imu_usart_read_mult_reg(uint8_t start_reg, uint8_t *data, uint8_t len)
{
	usart_send_blocking(USART1, 0xAA);
	usart_send_blocking(USART1, 0x01);
	usart_send_blocking(USART1, start_reg);
	usart_send_blocking(USART1, len);

	int res = usart_recv_blocking(USART1);
	int len_recv = 0;
	int ret = 0;
	int i = 0;

	if(res == 0xEE)
	{
		return -1;
	}
	else
	{
		len_recv = usart_recv_blocking(USART1);
		if(len_recv != len)
		{
			return -2;
		}
		else
		{
			for(i=0; i<len_recv; ++i)
			{
				data[i] = usart_recv_blocking(USART1);
			}
		}
	}
}

static int aux_imu_usart_write_single_reg(uint8_t reg, uint8_t val)
{
	usart_send_blocking(USART1, 0xAA);
	usart_send_blocking(USART1, 0x00);
	usart_send_blocking(USART1, reg);
	usart_send_blocking(USART1, 0x01);
	usart_send_blocking(USART1, val);
}

void get_aux_imu_euler(float *angles)
{
	// int16_t raw_values[3];

	union {
		uint8_t input[2];
		int16_t output;
	} bytes_to_signed_int16;

	// bytes_to_signed_int16.input[0] = aux_imu_read_reg(0x1c);
	// bytes_to_signed_int16.input[1] = aux_imu_read_reg(0x1d);

	// raw_values[0] = bytes_to_signed_int16.output;

	// bytes_to_signed_int16.input[0] = aux_imu_read_reg(0x1e);
	// bytes_to_signed_int16.input[1] = aux_imu_read_reg(0x1f);

	// raw_values[1] = bytes_to_signed_int16.output;

	// bytes_to_signed_int16.input[0] = aux_imu_read_reg(0x1a);
	// bytes_to_signed_int16.input[1] = aux_imu_read_reg(0x1b);

	// raw_values[2] = bytes_to_signed_int16.output;

	// angles[0] = (float)raw_values[0]/(float)16.0f;
	// angles[1] = (float)raw_values[1]/(float)16.0f;
	// angles[2] = (float)raw_values[2]/(float)16.0f;

	uint8_t imu_raw_data[6];

	aux_imu_usart_read_mult_reg(0x1a, imu_raw_data, 6);

	// Heading:

	bytes_to_signed_int16.input[0] = imu_raw_data[0];
	bytes_to_signed_int16.input[1] = imu_raw_data[1];

	angles[2] = (float)bytes_to_signed_int16.output/(float)16.0f;

	// Roll:

	bytes_to_signed_int16.input[0] = imu_raw_data[2];
	bytes_to_signed_int16.input[1] = imu_raw_data[3];

	angles[0] = (float)bytes_to_signed_int16.output/(float)16.0f;

	// Pitch:

	bytes_to_signed_int16.input[0] = imu_raw_data[4];
	bytes_to_signed_int16.input[1] = imu_raw_data[5];

	angles[1] = (float)bytes_to_signed_int16.output/(float)16.0f;
}

int main(void)
{
  init_serial_comm_datastructures();
	imu_i2c_bus_clear();
	_disable_interrupts();

		imu_scaled_data_struct imu_struct_scaled;

		set_system_clock();

		led_gpio_setup();
		systick_setup();
		usart2_setup();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		init_mission_timekeeper();
		initialize_imu(SCALE_2G, SCALE_1POINT9_GAUSS, SCALE_250_DPS, &imu_struct_scaled);

    while (1)
    {
      usart_send_blocking(USART2, 'a');
      gpio_toggle(LBLUE2);
      timekeeper_delay(1000U);
    }
    test_send_some_messages();
		
	_enable_interrupts();

	#ifdef ENABLE_PWM_TEST_SHELL

			/*
				Simple shell for testing PWM functionality:
			 */
			_disable_interrupts();

			QuadRotor_PWM_init();
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			
			_enable_interrupts();

			pwm_test_shell();

	#endif

	printf("Starting aux IMU\r\n");

	aux_imu_i2c_bus_setup();

	// usart1_setup();

	// timekeeper_delay(1000U);

	// // aux_imu_i2c_bus_setup();
	// // timekeeper_delay(1000U);

	// uint8_t id_data[4];
	// aux_imu_usart_read_mult_reg(0x00, id_data, 4);

	// printf("Chip ID: %x, Acc ID: %x, Mag ID: %x, Gyro ID: %x\r\n", id_data[0], id_data[1], id_data[2], id_data[3]);

	// timekeeper_delay(100U);

	// // aux_imu_usart_write_single_reg(0x3d, 0x00);

	// // timekeeper_delay(20U);

	// // aux_imu_usart_write_single_reg(0x3d, 0x08);

	// // uint8_t opr_mod = 0x08;

	// // write_i2c(I2C2, 0x28, 0x3D,	1, &opr_mod);

	// timekeeper_delay(100U);

	// int err_flags = aux_imu_usart_read_single_reg(0x39);

	// printf("Errors: %x\r\n", (uint8_t)err_flags);

	// float euler_angles[3];

	while (1)
	{
		gpio_toggle(LBLUE2);
	}

	return 0;
}
