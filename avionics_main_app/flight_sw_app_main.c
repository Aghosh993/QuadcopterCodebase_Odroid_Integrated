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
#include <vehicle_state_machine.h>
#include <px4flow_reader.h>
#include <sf10_reader.h>
#include <gimbal_pwm_controller.h>
#include <BNO055.h>
#include <simple_telemetry.h>

#define LBLUE2 GPIOE, GPIO12

sf10_sensor_data_handler *h0;

/*
	Approx active LOC (Lines of code: ~3200)
 */

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA9,10 			->			USART1 (OR I2C for auxiliary stuff)
	PA2,3 			-> 			USART2
	PE8,10,12,15 	-> 			LEDs
	PB6,7 			-> 			I2C1
	PA5,6,7 		-> 			SPI1 MISO,MOSI,CLK
	PE3 			-> 			SPI1 CS (user-controlled) for L3GD20
	PD3 			-> 			Timer2 Input capture
	PC6 			-> 			Timer3 Input capture
	PD12 			-> 			Timer4 Input capture
	PA15 			->			Timer8 Input capture
	PE9,11,13,PE14 	-> 			Timer1 PWM Output
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
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO12);
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
	usart_set_baudrate(USART2, 460800);//921600
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(USART2);
	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

#define USART3_USE_RX_ISR	1

static void usart3_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART3);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	#ifdef USART3_USE_RX_ISR
		// Unmask receive interrupt
		usart_enable_rx_interrupt(USART3);
		// Make sure the interrupt is routed through the NVIC
		nvic_enable_irq(NVIC_USART3_EXTI28_IRQ);
	#endif

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void uart4_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_UART4);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_af(GPIOC, GPIO_AF5, GPIO10 | GPIO11);

	/* Setup UART parameters. */
	usart_set_baudrate(UART4, 115200);//921600
	usart_set_databits(UART4, 8);
	usart_set_stopbits(UART4, USART_STOPBITS_1);
	usart_set_mode(UART4, USART_MODE_TX_RX);
	usart_set_parity(UART4, USART_PARITY_NONE);
	usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(UART4);
	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_UART4_EXTI34_IRQ);

	/* Finally enable the USART. */
	usart_enable(UART4);
}

#ifdef INCLUDE_PWM_TEST_SHELL

	static void pwm_test_shell(void)
	{
		float pwm_setval = 1.0f;
		char cmd;
		uint8_t err = 0U;
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
// static volatile uint8_t height_control_update_flag;

#ifdef NESTED_HEIGHT_CONTROLLER
	static volatile uint8_t vertical_acceleration_control_update_flag;
#endif

/*
	Get IMU data and complementary-filtered vehicle state estimate:
 */

static void get_vehicle_state(imu_scaled_data_struct *imu_data, 
								filtered_quadrotor_state *st_vector)
{
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
}

static void get_fused_imu_state(float *aux_imu_state)
{
	get_aux_imu_euler(aux_imu_state);
}

static void velocity_controller_update(float vel_cmd_x, float vel_cmd_y, float vel_x, float vel_y, float *roll_cmd, float *pitch_cmd)
{
	generate_attitude_commands(vel_x, vel_y, vel_cmd_x, vel_cmd_y, roll_cmd, pitch_cmd);
}

// #define USE_AUX_IMU_FUSION	1

static void vehicle_stabilization_outerloop_update(filtered_quadrotor_state *st_vector,
													imu_scaled_data_struct *imu_data,
													float *aux_imu_data,
													float roll_cmd_in,
													float pitch_cmd_in,
													float yaw_cmd_in,
													float *roll_rate_cmd_out,
													float *pitch_rate_cmd_out,
													float *yaw_rate_cmd_out)
{
	#ifndef USE_AUX_IMU_FUSION
		generate_rate_commands(st_vector, imu_data, roll_cmd_in, pitch_cmd_in, yaw_cmd_in, roll_rate_cmd_out, pitch_rate_cmd_out, yaw_rate_cmd_out);
	#endif
	#ifdef USE_AUX_IMU_FUSION
		filtered_quadrotor_state aux_state_vect;
		aux_state_vect.roll = aux_imu_data[0];
		aux_state_vect.pitch = aux_imu_data[1];
		aux_state_vect.yaw = aux_imu_data[2];
		generate_rate_commands(&aux_state_vect, imu_data, roll_cmd_in, pitch_cmd_in, yaw_cmd_in, roll_rate_cmd_out, pitch_rate_cmd_out, yaw_rate_cmd_out);
	#endif
}

static void vehicle_stabilization_innerloop_update(imu_scaled_data_struct *imu_data,
													float roll_rate_cmd_in,
													float pitch_rate_cmd_in,
													float yaw_rate_cmd_in,
													float throttle_value_in,
													double *motor_commands_out)
{
	rate_controller_update(motor_commands_out, imu_data, roll_rate_cmd_in, pitch_rate_cmd_in, yaw_rate_cmd_in, throttle_value_in);
}

static void get_rc_joystick_data(rc_joystick_data_struct *js_data_in, float *roll_command_out,
																		float *pitch_command_out,
																		float *yaw_command_out,
																		float *height_command_out,
																		float *mode_switch_value)
{
	get_rc_input_values(js_data_in);
	*height_command_out = ((js_data_in->vertical_channel_value)*0.5f)+0.5f;
	*roll_command_out = 1.0f*js_data_in->roll_channel_value;
	*pitch_command_out = -1.0f*js_data_in->pitch_channel_value;
	*yaw_command_out = -1.0f*js_data_in->yaw_channel_value;
	*mode_switch_value = js_data_in->mode_switch_channel_value;
}

typedef struct {
	/*Filter Constants:*/

	/*Time between filter updates in seconds:*/
	float filter_dt;
	/*Noise characteristic of LIDAR Altimeter:*/
	float r_lidar;

	/*========================================*/

	/*Filter Variables*/

	/*Filter state estimate (aka The Money :))*/
	float height_estimated;
	float vertical_velocity_estimated;

	/*Latest sensor data fed into the filter as of the last update cycle:*/
	float h_lidar_global_coords;
	float a_accel_global_coords;

	/*P matrix elements:*/
	float p11;
	float p12;
	float p21;
	float p22;

	/*G (Kalman Gain) matrix elements:*/
	float g11;
	float g21;
} height_kalman_data_struct;

static void height_kalman_struct_init(height_kalman_data_struct *str, float filter_dt_sec, float r_lidar)
{
	str->filter_dt = filter_dt_sec;
	str->r_lidar = r_lidar;

	str->height_estimated = 0.0f;
	str->vertical_velocity_estimated = 0.0f;

	str->h_lidar_global_coords = 0.059f;
	str->a_accel_global_coords = 0.0f;

	/*Initialize the P matrix to essentially an identity matrix to be on the safe side...?*/
	str->p11 = 100.0f;
	str->p12 = 0.0f;
	str->p21 = 0.0f;
	str->p22 = 10.0f;

	str->g11 = 0.0f;
	str->g21 = 0.0f;
}

static void height_kalman_update(height_kalman_data_struct *str, float lidar_height_measurement, float accelerometer_z_measurement, float vehicle_roll_deg, float vehicle_pitch_deg)
{
	/*Prediction intermediate variables:*/
	float p11_predicted, p12_predicted, p21_predicted, p22_predicted;
	float height_predicted, vertical_velocity_predicted;

	/*Predict:*/

	height_predicted = str->height_estimated + (str->vertical_velocity_estimated * str->filter_dt) + (0.50f * str->a_accel_global_coords * str->filter_dt * str->filter_dt);
	vertical_velocity_predicted = str->vertical_velocity_estimated + (str->a_accel_global_coords * str->filter_dt);

	p11_predicted = str->p11 + str->p21 * str->filter_dt + str->p12 * str->filter_dt + str->p22 * str->filter_dt * str->filter_dt;
	p12_predicted = str->p12 + str->p22 * str->filter_dt;
	p21_predicted = str->p21 + str->p22 * str->filter_dt;
	p22_predicted = str->p22;

	// Corariance prediction noise.. aka the Q matrix :)
	p11_predicted += 0.5f;
	p12_predicted += 0.5f;
	p21_predicted += 0.5f;
	p22_predicted += 0.5f;

	/*Update:*/
	float roll_cos = cos(vehicle_roll_deg * DEGREES_TO_RADIANS_CONVERSION_FACTOR);
	float pitch_cos = cos(vehicle_pitch_deg * DEGREES_TO_RADIANS_CONVERSION_FACTOR);
	float roll_pitch_cos_prod = roll_cos * pitch_cos;
	if(roll_pitch_cos_prod != 0.0f)
	{
		str->h_lidar_global_coords = lidar_height_measurement*(roll_cos * pitch_cos);
		str->a_accel_global_coords = (accelerometer_z_measurement/(roll_cos * pitch_cos))- 9.810f;	
	}

	str->g11 = p11_predicted/(p11_predicted + str->r_lidar);
	str->g21 = p21_predicted/(p11_predicted + str->r_lidar);

	float innovation = str->h_lidar_global_coords - height_predicted;
	str->height_estimated = height_predicted + str->g11 * innovation;
	str->vertical_velocity_estimated = vertical_velocity_predicted + str->g21 * innovation;

	str->p11 = p11_predicted - p11_predicted * str->g11;
	str->p12 = p12_predicted - p12_predicted * str->g11;
	str->p21 = -1.0f * str->g21 * p11_predicted + p21_predicted;
	str->p22 = -1.0f * str->g21 * p12_predicted + p22_predicted;
}

static float bno055_get_relative_heading(float raw_heading, float prior_heading)
{
	float delta_theta = raw_heading - prior_heading;
	if(delta_theta > 180.0f)
	{
		delta_theta -= 360.0f;
	}
	if(delta_theta < -180.0f)
	{
		delta_theta += 360.0f;
	}
	return delta_theta;
}

static float height_controller_thrust_offset(float rotor_dia_meters, float height_meters)
{
	float min_thrust_offset = 0.50f;
	float max_thrust_offset = 0.58f;
	float slope = (float)(max_thrust_offset - min_thrust_offset)/(float)(rotor_dia_meters * 0.65f);
	if(height_meters >= 0.0f && height_meters <= rotor_dia_meters * (float)0.65f)
	{
		return min_thrust_offset + (slope * height_meters);
	}
	if(height_meters > rotor_dia_meters * (float)0.65f)
	{
		return max_thrust_offset;
	}
	return min_thrust_offset;
}

typedef enum {
	MODE_MANUAL_CONTROL,
	MODE_TAKEOFF,
	MODE_XY_RELATIVE_POSITION_CONTROL,
	MODE_PX4FLOW_VELOCITY_CONTROL,
	MODE_AUTOMATIC_LANDING,
	MODE_ZERO_ATTITUDE_COMMANDS, // Set roll and pitch commands to 0 and height and yaw commands to whatever the last commands were for those axes
	MODE_EMERGENCY_ALL_PROPS_OFF // The Hail Mary, aka "Holy Shit" moment mode...
} vehicle_operational_mode;

void print_vehicle_mode(vehicle_operational_mode m)
{
	switch(m)
	{
		case MODE_MANUAL_CONTROL:
			printf("Vehicle moded into manual mode (User controlling Yaw, height, roll, pitch)\r\n");
			break;
		case MODE_TAKEOFF:
			printf("Vehicle moded into takeoff mode\r\n");
			break;
		case MODE_XY_RELATIVE_POSITION_CONTROL:
			printf("Vehicle moded into closed-loop XY relative positioning control, with user controlling height\r\n");
			break;
		case MODE_AUTOMATIC_LANDING:
			printf("Vehicle moded into RAPID automatic landing sequence\r\n");
			break;
		case MODE_EMERGENCY_ALL_PROPS_OFF:
			printf("Fire in the hole, we're going down!!! All motors commanded OFF!!!\r\n");
			break;
	}
}

// NEED TO TEST THIS CHANGE OUT VIA PRINTF!!!
#define SF10_UGV_THRESHOLD	0.18f // in meters, was 0.1f in last flight test
#define UGV_HEIGHT			0.50f // in meters

typedef enum {
	STATE_VEHICLE_ABOVE_GROUND,
	STATE_TRANSITIONING_TO_UGV,
	STATE_VEHICLE_ABOVE_UGV,
	STATE_TRANSITIONING_TO_GROUND
} vehicle_relative_height_tracker;

static float floating_pt_abs(float n)
{
	if(n > 0.0f)
	{
		return n;
	}
	return n * -1.0f;
}

float start_height;

static float get_compensated_sf10_data(vehicle_relative_height_tracker *tr, 
											float sf10_raw_measurement, float sf10_previous_raw_measurement,
											float current_height_cmd, float previous_height_cmd)
{
	switch(*tr)
	{
		// NEED TO ADDRESS POTENTIAL UNDEFINED CASES!!!
		case STATE_VEHICLE_ABOVE_GROUND:
			if(sf10_raw_measurement - sf10_previous_raw_measurement < -1.0f * SF10_UGV_THRESHOLD &&
					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
			{
				*tr = STATE_TRANSITIONING_TO_UGV;
				start_height = sf10_previous_raw_measurement;
				return sf10_raw_measurement;
			}
			return sf10_raw_measurement;
			break;
		case STATE_TRANSITIONING_TO_UGV:
			if(sf10_raw_measurement - start_height < -1.0f * SF10_UGV_THRESHOLD &&
					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
			{
				*tr = STATE_VEHICLE_ABOVE_UGV;
				return sf10_raw_measurement + UGV_HEIGHT;
			}
			else
			{
				*tr = STATE_VEHICLE_ABOVE_GROUND;
				return sf10_raw_measurement;
			}
			break;
		case STATE_VEHICLE_ABOVE_UGV:
			if(sf10_raw_measurement - sf10_previous_raw_measurement > SF10_UGV_THRESHOLD &&
					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
			{
				*tr = STATE_TRANSITIONING_TO_GROUND;
				start_height = sf10_previous_raw_measurement;
				return sf10_raw_measurement + UGV_HEIGHT;
			}
			return sf10_raw_measurement + UGV_HEIGHT;
			break;
		case STATE_TRANSITIONING_TO_GROUND:
			if(sf10_raw_measurement - start_height > SF10_UGV_THRESHOLD &&
					floating_pt_abs(current_height_cmd - previous_height_cmd) < SF10_UGV_THRESHOLD)
			{
				*tr = STATE_VEHICLE_ABOVE_GROUND;
				return sf10_raw_measurement;
			}
			else
			{
				*tr = STATE_VEHICLE_ABOVE_UGV;
				return sf10_raw_measurement + UGV_HEIGHT;
			}
			break;
		default:
			return sf10_raw_measurement;
			break;
	}
	return sf10_raw_measurement;
}

#define RAPID_DESCENT_INITIAL_HEIGHT		0.80f 		// In Meters
#define RAPID_DESCENT_VELOCITY				0.65f 		// In Meters*s^-1
#define RAPID_DESCENT_MOTOR_CUTOFF_HEIGHT	0.08f 		// In Meters

void automated_landing_sequence(float relative_time, float *height_setpoint_output, float actual_height)
{
	float h_setpoint = RAPID_DESCENT_INITIAL_HEIGHT - RAPID_DESCENT_VELOCITY * relative_time;
	if(actual_height < RAPID_DESCENT_MOTOR_CUTOFF_HEIGHT)
	{
		*height_setpoint_output = 0.0f;

		// We're done!! Disarm all systems and await shutoff/reset...
		disable_controller();
		QuadRotor_motor1_stop();
		QuadRotor_motor2_stop();
		QuadRotor_motor3_stop();
		QuadRotor_motor4_stop();
		while(1);
	}
	else
	{
		*height_setpoint_output = h_setpoint;
	}
}

void estimate_lateral_velocity(float* velocity_x_output, float *velocity_y_output, pxflow_flow_data_struct flow_data_input, float height_input, float current_roll_degrees, float current_pitch_degrees)
{
	static float last_roll_angle = 0.0f;
	static float last_pitch_angle = 0.0f;

	float d_roll = current_roll_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR - last_roll_angle;
	float d_pitch = current_pitch_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR - last_pitch_angle;

	float k1 = 1.0f;
	float k2 = 1.0f;

	// *velocity_x_output = k1 * ((float)flow_data_input.raw_x_flow * 3.940f * height_input - k2 * d_roll * 1.0f * height_input);
	// *velocity_y_output = k1 * ((float)flow_data_input.raw_y_flow * -3.940f * height_input - k2 * d_pitch * -1.0f * height_input);
	*velocity_x_output = flow_data_input.x_velocity;
	*velocity_y_output = flow_data_input.y_velocity;

	last_roll_angle = current_roll_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR;
	last_pitch_angle = current_pitch_degrees * DEGREES_TO_RADIANS_CONVERSION_FACTOR;
}

#define VELOCITY_CONTROL 	1
#define ENABLE_MOTORS		1

// #define VERBOSE_STARTUP		1

void px4flow_get_bias(int16_t* bias_x, int16_t* bias_y)
{
	int i = 0;

	int accum_x, accum_y;

	accum_x = 0;
	accum_y = 0;

	pxflow_flow_data_struct st;

	for(i = 0; i < 1000; ++i)
	{
		while(!px4flow_received_new_data());
		get_pxflow_flow_data(&st);
		accum_x += (int)st.raw_x_flow;
		accum_y += (int)st.raw_y_flow;
		// printf("%d %d\r\n", st.raw_x_flow, st.raw_y_flow);
	}
	*bias_x = (int16_t)((float)accum_x/(float)1000);
	*bias_y = (int16_t)((float)accum_y/(float)1000);
}

int main(void)
{
	vehicle_operational_mode vehicle_mode = MODE_MANUAL_CONTROL;
		// init_serial_comm_datastructures();
		// register_comm_callbacks();
	_disable_interrupts();

	init_simple_telemetry_lib();

	imu_scaled_data_struct imu_struct_scaled;

	rc_joystick_data_struct js;
	vehicle_state qr_state_variable = STATE_IMU_CAL;

	set_system_clock();

	led_gpio_setup();
	systick_setup();
	usart2_setup(); //Debug port
	usart3_setup(); //Px4flow
	uart4_setup(); // external IMU (BNO055)

	// #ifdef ENABLE_MOTORS
	// 	QuadRotor_PWM_init();
	// 	QuadRotor_motor1_setDuty(0.0f);
	// 	QuadRotor_motor2_setDuty(0.0f);
	// 	QuadRotor_motor3_setDuty(0.0f);
	// 	QuadRotor_motor4_setDuty(0.0f);
	// #endif
		
	setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
	setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

	print_vehicle_mode(vehicle_mode);

	#ifdef VERBOSE_STARTUP
		printf("Initialized PWM channels\r\n");
	#endif

	gimbal_init();
	gimbal_set_angles(0.0f, -45.0f);

	#ifdef VERBOSE_STARTUP
		printf("Initialized Gimbal\r\n");
	#endif

	#ifdef ENABLE_PWM_TEST_SHELL
		/*
			Simple shell for testing PWM functionality:
		 */
		pwm_test_shell();
	#endif

	#ifdef VERBOSE_STARTUP    	
		printf("Initializing timekeeping library...\r\n");
	#endif

	init_mission_timekeeper();

	#ifdef VERBOSE_STARTUP
		printf("Initializing IMU I2C bus...\r\n");
	#endif

	initialize_imu(SCALE_2G, SCALE_1POINT9_GAUSS, SCALE_250_DPS, &imu_struct_scaled);

	#ifdef VERBOSE_STARTUP
		printf("Setting up LIDAR Altimeter comms...\r\n");
	#endif

	sf10_hal_setup_stm32_serial();
	h0 = create_new_sf10_data_handler(100U, MAX_HEIGHT_SF10_A, send_byte_to_sf10A);

	#ifdef VERBOSE_STARTUP
		printf("Initializing Kalman height state estimator...\r\n");
	#endif

	height_kalman_data_struct height_estimator;
	height_kalman_struct_init(&height_estimator, 0.006f, 0.05f);

	#ifdef VERBOSE_STARTUP
		printf("Interrupts on!!\r\n");
	#endif

	_enable_interrupts();

	#ifdef VERBOSE_STARTUP
		printf("Performing IMU Bias Calibration...\r\n");
	#endif

	filtered_quadrotor_state st;
	do_bias_calculation(&imu_struct_scaled);

	#ifdef VERBOSE_STARTUP
		printf("Initializing Complementary Filter for on-board state estimation...\r\n");
	#endif

	init_comp_filter(&st);

	#ifdef VERBOSE_STARTUP
		printf("Setting up all timing flags for scheduled recurring events...\r\n");
	#endif

	uint8_t three_ms_flag = create_flag(3U);
	uint8_t six_ms_flag = create_flag(6U);

	#ifdef VELOCITY_CONTROL
		uint16_t velocity_control_interval_ms = (uint16_t)(LATERAL_VELOCITY_CONTROL_DT*1000.0f);
		uint8_t velocity_control_flag = create_flag(velocity_control_interval_ms);
	#endif

	uint8_t rc_update_flag = create_flag(24U);
	#ifndef ENABLE_MOTORS
		uint8_t dbg_output_flag = create_flag(160U);
	#endif
	#ifdef ENABLE_MOTORS
			uint8_t dbg_output_flag = create_flag(6U);
	#endif
	uint8_t sf10_trigger_flag = create_flag(36U);
	uint8_t external_imu_trigger_flag = create_flag(12U);
	uint8_t relative_positioning_controller_flag = create_flag(66U);

	#ifdef VERBOSE_STARTUP
		#ifndef ENABLE_MOTORS
			printf("RC Calibration disabled due to motors being disabled... skipping...\r\n");
		#endif
	#endif

	#ifdef ENABLE_MOTORS
		#ifdef VERBOSE_STARTUP
			printf("Calibrating RC Joysticks\r\n");
		#endif
		init_rc_inputs(&js);
	#endif

	#ifdef VERBOSE_STARTUP
		printf("Initializing PID controllers...\r\n");
	#endif

	controller_init_vars();
	set_controller_mode(MODE_ANGULAR_POSITION_CONTROL);
	// set_controller_mode(MODE_ANGULAR_RATE_CONTROL);
	
	initialize_vehicle_state_machine(&qr_state_variable);

	float vel_x_cmd, vel_y_cmd, vel_z_cmd;
	float roll_cmd_js, pitch_cmd_js;
	float roll_cmd, pitch_cmd, yaw_cmd, height_cmd, mode_switch_input;
	float roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd;
	double motor_commands[4];

	roll_cmd_js = 0.0f;
	pitch_cmd_js = 0.0f;

	roll_cmd = 0.0f;
	pitch_cmd = 0.0f;
	yaw_cmd = 0.0f;
	height_cmd = 0.0f;
	mode_switch_input = 0.0f;

	roll_rate_cmd = 0.0f;
	pitch_rate_cmd = 0.0f;
	yaw_rate_cmd = 0.0f;

	vel_x_cmd = 0.0f;
	vel_y_cmd = 0.0f;

	motor_commands[0] = 0.0f;
	motor_commands[1] = 0.0f;
	motor_commands[2] = 0.0f;
	motor_commands[3] = 0.0f;

	float aux_state_vect[3];
	aux_state_vect[0] = 0.0f;
	aux_state_vect[1] = 0.0f;
	aux_state_vect[2] = 0.0f;

	pxflow_flow_data_struct flow_data;
	imu_data bno055_data;

	bno055_data.roll = 0.0f;
	bno055_data.pitch = 0.0f;
	bno055_data.heading = 0.0f;

	bno055_data.gyro_x = 0.0f;
	bno055_data.gyro_y = 0.0f;
	bno055_data.gyro_z = 0.0f;

	bno055_data.quaternion_x = 0.0f;
	bno055_data.quaternion_y = 0.0f;
	bno055_data.quaternion_z = 0.0f;

	float h_sf10 = 0.0f;
	float sf10_vertical_bias = 0.059f; // In meters, describes how far off the ground by default the SF10 is when mounted on the vehicle chassis
	float gnd_effect_bias_term = 0.0f;

	float height_controller_throttle_cmd = 0.0f;

	#ifdef VERBOSE_STARTUP
		printf("Initializing BNO055 IMU... switching to and setting config mode\r\n");
	#endif
	
	BNO055_write_single_register(0x3d, 0x00);
	timekeeper_delay(20U);
	BNO055_write_single_register(0x3d, 0xC); // 0x8 = failsafe for IMU-only fusion mode
	timekeeper_delay(200U);

	#ifdef VERBOSE_STARTUP
		printf("Config done, triggering measurements from BNO055 IMU...\r\n");
	#endif

	BNO055_trigger_get_data();

	float user_yaw_command = 0.0f;
	float yaw_integration_deadband = 0.05f;
	float yaw_cmd_integration_multiplier = 0.1f;

	#ifdef VELOCITY_CONTROL
		float vel_err_x, vel_err_y;
		float pos_err_x = 0.0f;
		float pos_err_y = 0.0f;
		float pos_err_max = 2.0f;

		float vel_ctrl_output_max = 15.0f * 3.14159f/180.0f;

		uint8_t lateral_vel_control_enabled = 1;
	#endif

	float previous_sf10_raw_height = 0.0f;
	float previous_user_height_cmd = 0.0f;
	float h_sf10_ugv_compensated = 0.0f;

	float kalman_filter_height_input = 0.0f;

	vehicle_relative_height_tracker relative_height_compensator = STATE_VEHICLE_ABOVE_GROUND;

	float height_error = 0.0f;
	float height_pid_err_accum = 0.0f;
	float angular_adjustment = 0.0f;
	float height_pid_adj = 0.0f;
	float height_pid_max_adj = 0.275f;
	float max_height_cmd = 4.0f;//1.250f;

	float telemetry_stream_contents[TELEMETRY_N_FLOATS_TO_SEND];
	time_val system_time;

	uint8_t start_landing = 0U;
	float landing_start_time = 0.0f;

	odroid_packet_t pixel_data;
	pixel_data.px = 320.0f;
	pixel_data.py = 240.0f;
	pixel_data.vx = 0.0f;
	pixel_data.vy = 0.0f;
	pixel_data.not_found = 1U;

	int odroid_read_result = -1;

	float rel_pos_ctrl_roll_cmd = 0.0f;
	float rel_pos_ctrl_pitch_cmd = 0.0f;

	float rel_pos_ctrl_roll_accumulated_error = 0.0f;
	float rel_pos_ctrl_pitch_accumulated_error = 0.0f;

	float rel_pos_ctrl_max_output = 0.13f;

	#ifdef VERBOSE_STARTUP
		printf("Please move height control joystick axis down to LOW endstop to arm vehicle...\r\n");
	#endif

	while(height_cmd*max_height_cmd > 0.06f)
	{
		get_rc_joystick_data(&js, &roll_cmd, &pitch_cmd, &yaw_cmd, &height_cmd, &mode_switch_input);
	}

	#ifdef ENABLE_MOTORS
		QuadRotor_PWM_init();
		QuadRotor_motor1_setDuty(0.0f);
		QuadRotor_motor2_setDuty(0.0f);
		QuadRotor_motor3_setDuty(0.0f);
		QuadRotor_motor4_setDuty(0.0f);

		enable_controller();

		QuadRotor_motor1_start();
		QuadRotor_motor2_start();
		QuadRotor_motor3_start();
		QuadRotor_motor4_start();

		#ifdef VERBOSE_STARTUP
			printf("Arming condition statisfied, vehicle is armed! Ensure all people are at least 15 feet from vehicle!\r\n");
		#endif

		timekeeper_delay(1000U);
	#endif

	float px4flow_compensated_vel_x = 0.0f;
	float px4flow_compensated_vel_y = 0.0f;

	float optical_flow_roll_cmd = 0.0f;
	float optical_flow_pitch_cmd = 0.0f;

	#ifdef VERBOSE_STARTUP
		printf("Obtaining Px4flow flow sensor bias...\r\n");
	#endif

	int16_t flow_bias_x, flow_bias_y;
	flow_bias_x = 0;
	flow_bias_y = 0;
	// px4flow_get_bias(&flow_bias_x, &flow_bias_y);

	#ifdef VERBOSE_STARTUP
		printf("Bias values... x: %d, y: %d\r\n", flow_bias_x, flow_bias_y);
	#endif	

	#ifdef VERBOSE_STARTUP
		printf("Ready for flight!! Proceeding to run main loop...\r\n");
	#endif

	while (1)
	{
	    if(get_flag_state(three_ms_flag) == STATE_PENDING)
	    {
	    	reset_flag(three_ms_flag);

	    	get_vehicle_state(&imu_struct_scaled, &st);

			#ifdef OPENLOOP_HEIGHT_CONTROL
				vehicle_stabilization_innerloop_update(&imu_struct_scaled, roll_rate_cmd, 
																			pitch_rate_cmd, 
																			yaw_rate_cmd,
																			height_cmd,
																			motor_commands);
			#endif
			#ifdef CLOSEDLOOP_HEIGHT_CONTROL
				vehicle_stabilization_innerloop_update(&imu_struct_scaled, roll_rate_cmd, 
																			pitch_rate_cmd, 
																			yaw_rate_cmd,
																			height_controller_throttle_cmd,
																			motor_commands);
			#endif
			#ifdef ENABLE_MOTORS
				QuadRotor_set_all_motors(motor_commands);
			#endif

	    	gpio_toggle(LBLUE2);

	    	if(get_flag_state(six_ms_flag) == STATE_PENDING)
	    	{
	    		reset_flag(six_ms_flag);

	    		height_kalman_update(&height_estimator, kalman_filter_height_input, st.vertical_dynamic_acceleration_post_lpf, st.roll, st.pitch);

		    	if(start_landing == 1U)
		    	{
		    		automated_landing_sequence((float)get_mission_time_as_millis()*(float)0.001f - landing_start_time, &height_cmd, height_estimator.height_estimated);
		    	}

				// Compute height error in meters:
				height_error = height_cmd - height_estimator.height_estimated;
				// Compute height accumulated error in meter*seconds:
				height_pid_err_accum += height_error * HEIGHT_CONTROL_DT;

				// Saturate the height accumulated error integral:
				if(height_pid_err_accum < -1.0f)
				{
					height_pid_err_accum = -1.0f;
				}
				if(height_pid_err_accum > 1.0f)
				{
					height_pid_err_accum = 1.0f;
				}

				// Compute height PID adjustment:
				height_pid_adj = (0.75f * height_error) + (height_pid_err_accum * 0.01f) + (-0.41f * height_estimator.vertical_velocity_estimated);
				
				// Saturate height PID adjustment:
				if(height_pid_adj < -1.0f * height_pid_max_adj)
				{
					height_pid_adj = -1.0f * height_pid_max_adj;
				}
				if(height_pid_adj > height_pid_max_adj)
				{
					height_pid_adj = height_pid_max_adj;
				}

				// height_controller_throttle_cmd = OPENLOOP_MOTOR_MIN_THRUST + height_pid_adj;

				// Compute height PID net throttle command (i.e. value to be sent to all 4 motors on vehicle as a net bias value)
				// by adding feed-forward term (based on current height, and acquired from lookup table/interpolation)
				// and feed-back term (i.e. PID adjustment computed above):
				height_controller_throttle_cmd = height_controller_thrust_offset(0.3048f, height_estimator.height_estimated) + height_pid_adj;

				// Compensate for roll and pitch of platform reducing effective lift vector:
				// angular_adjustment = cosf(st.roll * DEGREES_TO_RADIANS_CONVERSION_FACTOR) * cosf(st.pitch * DEGREES_TO_RADIANS_CONVERSION_FACTOR);

				// We really shouldn't have a cosine loss of over 15% if you run the numbers for roughly a 15-20-degree roll AND pitch combined...
				// We also better have a net positive thrust command:
				// if(angular_adjustment > 0.85f && angular_adjustment <= 1.0f)
				// {
				// 	height_controller_throttle_cmd = (float)height_controller_throttle_cmd / (float)angular_adjustment;
				// }

				vehicle_stabilization_outerloop_update(&st, &imu_struct_scaled, aux_state_vect, roll_cmd, pitch_cmd, user_yaw_command,
														&roll_rate_cmd, &pitch_rate_cmd, &yaw_rate_cmd);

	    	}
	    }

		if(get_flag_state(rc_update_flag) == STATE_PENDING)
		{
			reset_flag(rc_update_flag);

			get_rc_joystick_data(&js, &roll_cmd_js, &pitch_cmd_js, &yaw_cmd, &height_cmd, &mode_switch_input);
			height_cmd *= max_height_cmd;

			if(yaw_cmd < -1.0f*yaw_integration_deadband || yaw_cmd > yaw_integration_deadband)
			{
				user_yaw_command += -1.0f * yaw_cmd * yaw_cmd_integration_multiplier;
			}

			if(user_yaw_command < -2.0f * 3.14159f)
			{
				user_yaw_command = -2.0 * 3.14159f;
			}
			if(user_yaw_command > 2.0f * 3.14159f)
			{
				user_yaw_command = 2.0f * 3.14159f;
			}

			if(js.vertical_channel_validity == CHANNEL_INVALID || js.roll_channel_validity == CHANNEL_INVALID ||
				js.pitch_channel_validity == CHANNEL_INVALID || js.yaw_channel_validity == CHANNEL_INVALID ||
				js.mode_switch_channel_validity == CHANNEL_INVALID)
			{
				vehicle_mode = MODE_EMERGENCY_ALL_PROPS_OFF;
			}

			// Automated flight roll and pitch commands go here:
			if(get_ch5_mode(js) == MODE_NORMAL && vehicle_mode != MODE_EMERGENCY_ALL_PROPS_OFF)
			{
				vehicle_mode = MODE_PX4FLOW_VELOCITY_CONTROL;
			}

			if(get_ch5_mode(js) == MODE_FAILSAFE && vehicle_mode != MODE_EMERGENCY_ALL_PROPS_OFF)
			{
				vehicle_mode = MODE_MANUAL_CONTROL;
			}
		}

		switch(vehicle_mode)
		{
			case MODE_MANUAL_CONTROL:
				roll_cmd = roll_cmd_js;
				pitch_cmd = pitch_cmd_js;
				kalman_filter_height_input = h_sf10; // Use failsafe of feeding SF10 height data directly into Kalman Filter in failsafe mode
				break;
			case MODE_PX4FLOW_VELOCITY_CONTROL:
				vel_x_cmd = 0.0f;//0.50f * roll_cmd_js;
				vel_y_cmd = 0.0f;//0.50f * pitch_cmd_js;

				roll_cmd = optical_flow_roll_cmd;
				// pitch_cmd = optical_flow_pitch_cmd;
				pitch_cmd = pitch_cmd_js;
				
				kalman_filter_height_input = h_sf10;//h_sf10_ugv_compensated; // Use UGV-compensated height data in automatic control mode
				break;
			case MODE_XY_RELATIVE_POSITION_CONTROL:
				roll_cmd = -1.0f * rel_pos_ctrl_roll_cmd;
				pitch_cmd = -1.0f * rel_pos_ctrl_pitch_cmd;
				break;
			case MODE_EMERGENCY_ALL_PROPS_OFF:
				disable_controller();
				QuadRotor_motor1_stop();
				QuadRotor_motor2_stop();
				QuadRotor_motor3_stop();
				QuadRotor_motor4_stop();
				break;
			default:
				disable_controller();
				QuadRotor_motor1_stop();
				QuadRotor_motor2_stop();
				QuadRotor_motor3_stop();
				QuadRotor_motor4_stop();
				while(1);
				break;
		}

		#ifdef VELOCITY_CONTROL
			if(get_flag_state(velocity_control_flag) == STATE_PENDING)
			{
				reset_flag(velocity_control_flag);

				if(lateral_vel_control_enabled)
				{
					vel_err_x = flow_data.x_velocity - vel_x_cmd;
					vel_err_y = flow_data.y_velocity - vel_y_cmd;

					if(height_estimator.height_estimated < 0.20f)
					{
						pos_err_x = 0.0f;
						pos_err_y = 0.0f;
					}
					else
					{
						pos_err_x += vel_err_x * LATERAL_VELOCITY_CONTROL_DT;
						pos_err_y += vel_err_y * LATERAL_VELOCITY_CONTROL_DT;
					}

					if(pos_err_x > pos_err_max)
					{
						pos_err_x = pos_err_max;
					}
					if(pos_err_x < -1.0f * pos_err_max)
					{
						pos_err_x = -1.0f * pos_err_max;
					}

					if(pos_err_y > pos_err_max)
					{
						pos_err_y = pos_err_max;
					}
					if(pos_err_y < -1.0f * pos_err_max)
					{
						pos_err_y = -1.0f * pos_err_max;
					}

					optical_flow_roll_cmd = -1.0f*(2.75f * vel_err_x + 0.0f * pos_err_x);
					optical_flow_pitch_cmd = 2.50f * vel_err_y + 0.0f * pos_err_y;

					if(optical_flow_roll_cmd > vel_ctrl_output_max)
					{
						optical_flow_roll_cmd = vel_ctrl_output_max;
					}
					if(optical_flow_roll_cmd < -1.0f * vel_ctrl_output_max)
					{
						optical_flow_roll_cmd = -1.0f * vel_ctrl_output_max;
					}

					if(optical_flow_pitch_cmd > vel_ctrl_output_max)
					{
						optical_flow_pitch_cmd = vel_ctrl_output_max;
					}
					if(optical_flow_pitch_cmd < -1.0f * vel_ctrl_output_max)
					{
						optical_flow_pitch_cmd = -1.0f * vel_ctrl_output_max;
					}
				}
			}
		#endif

	    // Trigger external IMU:
	    if(get_flag_state(external_imu_trigger_flag) == STATE_PENDING)
	    {
	    	reset_flag(external_imu_trigger_flag);
	    	BNO055_trigger_get_data();
	    }

	    // Trigger acquisition of LIDAR Height sensor data:
		if(get_flag_state(sf10_trigger_flag) == STATE_PENDING)
		{
			reset_flag(sf10_trigger_flag);
			request_sf10_sensor_update(h0);	
		}

		if(get_flag_state(relative_positioning_controller_flag) == STATE_PENDING)
		{
			reset_flag(relative_positioning_controller_flag);

			if(pixel_data.not_found == 0U)
			{
				rel_pos_ctrl_roll_cmd = 0.001f * (pixel_data.px - 320.0f);// + (-0.000030f * pixel_data.vx);
				rel_pos_ctrl_pitch_cmd = 0.001f * (pixel_data.py - 240.0f);// + (-0.000030f * pixel_data.vx);

				if(rel_pos_ctrl_roll_cmd > rel_pos_ctrl_max_output)
				{
					rel_pos_ctrl_roll_cmd = rel_pos_ctrl_max_output;
				}
				if(rel_pos_ctrl_roll_cmd < -1.0f * rel_pos_ctrl_max_output)
				{
					rel_pos_ctrl_roll_cmd = -1.0f * rel_pos_ctrl_max_output;
				}

				if(rel_pos_ctrl_pitch_cmd > rel_pos_ctrl_max_output)
				{
					rel_pos_ctrl_pitch_cmd = rel_pos_ctrl_max_output;
				}
				if(rel_pos_ctrl_pitch_cmd < -1.0f * rel_pos_ctrl_max_output)
				{
					rel_pos_ctrl_pitch_cmd = -1.0f * rel_pos_ctrl_max_output;
				}
			}
			else
			{
				rel_pos_ctrl_roll_cmd = 0.0f;
				rel_pos_ctrl_pitch_cmd = 0.0f;
			}
		}

		// Print debug output:
		if(get_flag_state(dbg_output_flag) == STATE_PENDING)
		{
			reset_flag(dbg_output_flag);

			system_time = get_mission_time();

			telemetry_stream_contents[0] = (float)system_time.seconds + (float)system_time.ms * (float)0.001f;

			telemetry_stream_contents[1] = st.roll;
			telemetry_stream_contents[2] = st.pitch;
			telemetry_stream_contents[3] = st.yaw;
			telemetry_stream_contents[4] = height_estimator.height_estimated;

			telemetry_stream_contents[5] = optical_flow_roll_cmd;// roll_cmd;
			telemetry_stream_contents[6] = pitch_cmd;
			telemetry_stream_contents[7] = height_cmd;

			telemetry_stream_contents[8] = (float)motor_commands[MOTOR_1];
			telemetry_stream_contents[9] = (float)motor_commands[MOTOR_2];
			telemetry_stream_contents[10] = (float)motor_commands[MOTOR_3];
			telemetry_stream_contents[11] = (float)motor_commands[MOTOR_4];

			#ifdef ENABLE_MOTORS
				send_n_floats(telemetry_stream_contents);
			#endif

			#ifndef ENABLE_MOTORS
				printf("Dbg: Pos Err %f\r\n", pos_err_x);
			#endif
		}

		/*
		 * Asynchronous events due to sensor data input:
		 */

	    if(new_odroid_message_available())
	    {
	    	odroid_read_result = get_latest_odroid_message(&pixel_data);
	    }

		// Get Px4Flow Sensor height and optical flow data:
	    if (px4flow_received_new_data())
	    {
			get_pxflow_flow_data(&flow_data);
			flow_data.raw_x_flow -= flow_bias_x;
			flow_data.raw_y_flow -= flow_bias_y;
			float h_pxflow = get_pxflow_height();

			if(flow_data.quality > 200U)
			{
				estimate_lateral_velocity(&px4flow_compensated_vel_x, &px4flow_compensated_vel_y, flow_data, height_estimator.height_estimated, st.roll, st.pitch);
			}
	    }

	    // Get LIDAR Altimeter data:
	    if (sf10_received_new_data())
	    {
			h_sf10 = get_last_sf10_sensor_height(h0);

			h_sf10_ugv_compensated = get_compensated_sf10_data(&relative_height_compensator,
																h_sf10, previous_sf10_raw_height, 
																height_cmd, previous_user_height_cmd);
			previous_sf10_raw_height = h_sf10;
			previous_user_height_cmd = height_cmd;
	    }

	    // Get BNO055 auxiliary IMU data:
	    if (BNO055_received_new_data())
	    {
	    	BNO055_get_imu_data(&bno055_data);
	    	st.yaw = bno055_data.heading;
	    }
	}
	return 0;
}
