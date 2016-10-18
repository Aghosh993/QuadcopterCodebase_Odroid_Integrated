#ifndef GIMBAL_PWM_CONTROLLER_H
#define GIMBAL_PWM_CONTROLLER_H 1

#define HEADING_SPEED_MAX -100.0
#define HEADING_SPEED_MIN 100.0

#define PITCH_LOWER_LIMIT -90.0
#define PITCH_UPPER_LIMIT 90.0

static void setup_timer16_pwm(void);
static void setup_timer17_pwm(void);

static void gimbal_pwm_init(void);
static void gimbal_pwm_start(void);
static void gimbal_pwm_stop(void);

static void gimbal_heading_pwm_start(void);
static void gimbal_pitch_pwm_start(void);
static void gimbal_mode_pwm_start(void);

static void gimbal_heading_pwm_stop(void);
static void gimbal_pitch_pwm_stop(void);
static void gimbal_mode_pwm_stop(void);

static void set_heading_rate(float rates);
static void set_pitch(float angle);
static void set_mode(int mode);

void gimbal_init(void);
void gimbal_set_angles(float heading_rate, float pitch);

#endif