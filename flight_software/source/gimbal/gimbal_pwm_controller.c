#include "QuadRotor_PWM_hal.h"

#include "gimbal_pwm_controller.h"

static void setup_timer16_pwm(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);

	gpio_set_af(GPIOB, GPIO_AF1, GPIO8);

	timer_reset(TIM16);
	rcc_periph_clock_enable(RCC_TIM16);

	timer_continuous_mode(TIM16);

	timer_set_mode(TIM16, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM16, TIM_OC1, TIM_OCM_PWM1);

	timer_set_period(TIM16, 64000);
	timer_set_prescaler(TIM16, 19); // for 50 Hz, this needs to be 1 for 500 Hz output

	timer_set_oc_value(TIM16, TIM_OC1, 0);

	timer_enable_preload(TIM16);

	timer_set_oc_polarity_high(TIM16, TIM_OC1);
	timer_enable_oc_output(TIM16, TIM_OC1);
	TIM_BDTR(TIM16) |= TIM_BDTR_MOE;

	timer_generate_event(TIM16, TIM_EGR_UG);

	timer_enable_counter(TIM16);
}

static void setup_timer17_pwm(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO5);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);

	gpio_set_af(GPIOB, GPIO_AF10, GPIO5);

	timer_reset(TIM17);
	rcc_periph_clock_enable(RCC_TIM17);

	timer_continuous_mode(TIM17);

	timer_set_mode(TIM17, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM17, TIM_OC1, TIM_OCM_PWM1);

	timer_set_period(TIM17, 64000);
	timer_set_prescaler(TIM17, 19); // for 50 Hz, this needs to be 1 for 500 Hz output

	timer_set_oc_value(TIM17, TIM_OC1, 0);

	timer_enable_preload(TIM17);

	timer_set_oc_polarity_high(TIM17, TIM_OC1);
	timer_enable_oc_output(TIM17, TIM_OC1);
	TIM_BDTR(TIM17) |= TIM_BDTR_MOE;

	timer_generate_event(TIM17, TIM_EGR_UG);

	timer_enable_counter(TIM17);
}

static void gimbal_pwm_init(void)
{
	setup_timer16_pwm();
	setup_timer17_pwm();
}

static void gimbal_pwm_start(void)
{
	gimbal_heading_pwm_start();
	gimbal_pitch_pwm_start();
	gimbal_mode_pwm_start();
}

static void gimbal_pwm_stop(void)
{
	gimbal_heading_pwm_stop();
	gimbal_pitch_pwm_stop();
	gimbal_mode_pwm_stop();
}

static void gimbal_heading_pwm_start(void)
{
	timer_set_oc_value(TIM16, TIM_OC1, 0);
	timer_enable_oc_output(TIM16, TIM_OC1);
}

static void gimbal_pitch_pwm_start(void)
{
	timer_set_oc_value(TIM17, TIM_OC1, 0);
	timer_enable_oc_output(TIM17, TIM_OC1);
}

static void gimbal_mode_pwm_start(void)
{
	timer_set_oc_value(TIM16, TIM_OC1, 0);
	timer_enable_oc_output(TIM16, TIM_OC1);
}

static void gimbal_heading_pwm_stop(void)
{
	timer_set_oc_value(TIM16, TIM_OC1, 0);
	timer_disable_oc_output(TIM16, TIM_OC1);
}

static void gimbal_pitch_pwm_stop(void)
{
	timer_set_oc_value(TIM17, TIM_OC1, 0);
	timer_disable_oc_output(TIM17, TIM_OC1);
}

static void gimbal_pwm_mode_stop(void)
{
	timer_set_oc_value(TIM16, TIM_OC1, 0);
	timer_disable_oc_output(TIM16, TIM_OC1);
}

/*
	The PWM will control the speed of the heading rotation.
	Minimum speed to move: ~20%
	100% speed: 235 degrees in 7 sec: 33.57 degrees/sec/
*/
static void set_heading_rate(float rate)
{
	float duty = 0.05*(1+(rate - HEADING_SPEED_MIN)/(HEADING_SPEED_MAX - HEADING_SPEED_MIN));
	uint32_t oc_value_scaled = (uint32_t)((float)(64000)*duty);
	if(duty >= MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM16, TIM_OC1, oc_value_scaled);
	}
}

static void set_pitch(float angle)
{
	float duty = 0.05*(1+(angle - PITCH_LOWER_LIMIT)/(PITCH_UPPER_LIMIT - PITCH_LOWER_LIMIT));
	uint32_t oc_value_scaled = (uint32_t)((float)(64000)*duty);
	if(duty >= MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM17, TIM_OC1, oc_value_scaled);
	}
}

/*
	Modes:
		1	->	Heading lock mode
		2	->	Heading and pitch follow mode
		3	->	Heading follow mode
*/
static void set_mode(int mode)
{
	float duty = 0.025 + 0.025*mode;
	uint32_t oc_value_scaled = (uint32_t)((float)(64000)*duty);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM16, TIM_OC1, oc_value_scaled);
	}
}

void gimbal_init(void)
{
	gimbal_pwm_init();
	gimbal_pwm_start();

	set_mode(1); // Heading lock mode
	gimbal_set_angles(0, 0);
}

void gimbal_set_angles(float heading_rate, float pitch)
{
	#if HEADING_CONTROL
		set_heading_rate(heading_rate);
	#endif

	set_pitch(pitch);
}
