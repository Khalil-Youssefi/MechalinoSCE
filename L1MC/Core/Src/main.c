/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	TIM_HandleTypeDef *tim_r;
	uint32_t           ch_r;
	TIM_HandleTypeDef *tim_l;
	uint32_t           ch_l;
	uint16_t           pwm_l;
	uint16_t           pwm_r;
} Motors;
typedef enum {
	GOTO_IDLE = 0,
	GOTO_ROTATE,
	GOTO_DRIVE,
	GOTO_BACKOFF,
	GOTO_DONE,
	GOTO_INACTIVE
} goto_state_t;

#define IR_SENSOR_COUNT 3u

typedef struct {
	uint32_t frame_sequence;
	uint32_t updated_ms;
	uint16_t adc_mv[IR_SENSOR_COUNT];
	uint8_t raw_mask;        // sensors above threshold in the newest frame
	uint8_t detected_mask;   // debounced sensor detections
	uint8_t robot_mask;      // detections whose projected point is near a fresh peer
	uint8_t static_mask;     // detections which should be added to the obstacle map
	float hit_x[IR_SENSOR_COUNT];
	float hit_y[IR_SENSOR_COUNT];
	int8_t cell_r[IR_SENSOR_COUNT];
	int8_t cell_c[IR_SENSOR_COUNT];
} obstacle_status_t;

typedef enum {
	CAMERA_IDLE = 0,
	CAMERA_SETTLING,
	CAMERA_WAITING
} camera_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Motors PWM values for forward/backward motion
#define MOTOR_PWM_STOP 1500
#define MOTOR_PWM_MAX_BACKWARD 1300
#define MOTOR_PWM_MAX_FORWARD 1700
#define FWD_L      (MOTOR_PWM_MAX_FORWARD)
#define FWD_R      (MOTOR_PWM_MAX_BACKWARD)

// Proportional rotation avoids overshooting and oscillating around the target.
#define ROT_PWM_MIN_DELTA       140.0f
#define ROT_PWM_MAX_DELTA       200.0f
#define ROT_PWM_PER_RAD         60.0f
#define GOTO_ROTATE_TIMEOUT_MS  5000u

// Encoder
#define ENCODER_RES 20

// Distance sensors
// Sensors direction relative to robot forward direction
#define S0_OFF_RAD  (0.0f)                                       // channel 0: front
#define S1_OFF_RAD  (DEG2RAD(-45.0f))                             // channel 1: front-right
#define S2_OFF_RAD  (DEG2RAD(+45.0f))                             // channel 7: front-left

#define IRD_NUM_SAMPLES 15

// DMUX control pins
#define DMUX_EN  GPIO_PIN_1
#define DMUX_A_PIN GPIO_PIN_3
#define DMUX_B_PIN GPIO_PIN_4
#define DMUX_C_PIN GPIO_PIN_5
#define DMUX_PORT GPIOB

// geometry
#define WHEEL_RADIUS_M 0.0225f   // [m]  wheel radius (2.25 cm)
#define WHEEL_BASE_M   0.14f   // [m]  distance between wheel centers

// cam pos receiving parameters
#define POS_LPF_ALPHA  0.8f
#define POS_WAIT_MS    1000u
#define STOP_SETTLE_MS  150u

// Goto xy thresholds
#define GOTO_THETA_OK_RAD        (5.0f * (float)M_PI / 180.0f)
#define GOTO_THETA_DRIVE_MAX_RAD (12.0f * (float)M_PI / 180.0f)
#define GOTO_DIST_OK_M           (0.02f)
#define INACTIVE_RECHECK_MS       500u
#define FINAL_CELL_DISTANCE_TIE_M (0.005f)

// grid parameters
#define CELL   0.15f
#define X0     0.18f // center of grid cell (0, 0) in arena coordinates
#define Y0     0.28f // center of grid cell (0, 0) in arena coordinates
#define COLS   11
#define ROWS   8

int path_r[ROWS*COLS];
int path_c[ROWS*COLS];
int path_len = 0;
int path_idx = 0;

// grid special values
#define INVALID_POS -1000.0f
#define MAX_VISIT_AND_PENALTY_COUNT 1000.0f

// SCE fitness parameters. The paper reports stable behavior for kappa > mu > lambda.
#define SCE_KAPPA              20u
#define SCE_MU                 4u
#define SCE_LAMBDA             2u
#define SCE_HEADING_WEIGHT     2.0f
#define SCE_PENALTY_DECAY      0.9f
#define SCE_FITNESS_TIE_EPS    1.0e-6f

// inter-swarm communication
#define MID 15                                                    // Mechalino ID (MID)
#define MAX_OTHER_ROBOTS 10                                      // maximum tracked peers
#define INVALID_MID 222
#define OPOS_QUEUE_DEPTH 64                                      // absorbs simultaneous peer broadcast bursts
#define OPOS_MESSAGE_SIZE 128                                    // 8x11 M1 packets are below 100 bytes
#define PEER_POSE_TIMEOUT_MS 2000u

// obstacle avoidance params
#define OBSTACLE_DIST_M       0.17f
#define OBSTACLE_TH0_MV       1600u                               // front trigger; lower than both sides
#define OBSTACLE_SIDE_TH_MV   1800u                               // symmetric +/-45-degree trigger
#define OBSTACLE_TH1_MV        OBSTACLE_SIDE_TH_MV                // front-right (DMUX channel 1)
#define OBSTACLE_TH2_MV        OBSTACLE_SIDE_TH_MV                // front-left (DMUX channel 7)
#define OBSTACLE_CLEAR_TH0_MV 1400u                               // front release
#define OBSTACLE_SIDE_CLEAR_MV 1600u                              // symmetric +/-45-degree release
#define OBSTACLE_CLEAR_TH1_MV OBSTACLE_SIDE_CLEAR_MV              // front-right release
#define OBSTACLE_CLEAR_TH2_MV OBSTACLE_SIDE_CLEAR_MV              // front-left release
#define OBSTACLE_CONFIRM_FRAMES 3u
#define OBSTACLE_MAP_CONFIRM_MS 100u                              // same static sensor/cell must persist before mapping
#define OBSTACLE_STATUS_STALE_MS 100u
#define OBSTACLE_BACKOFF_MS   800u

#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)

#define ROBOT_AS_OBS_GATE_M  0.18f   // distance threshold to treat hit as another robot
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Motors
Motors motors;

// Encoder
volatile int32_t encoder_right_count = 0;
volatile int32_t encoder_left_count = 0;

// Distance sensors
#define N_IR_ADC_CHANNELS 8u
volatile uint16_t adc_buffer[IRD_NUM_SAMPLES];
volatile uint16_t adc_readings_off[N_IR_ADC_CHANNELS];
volatile uint16_t adc_readings_on[N_IR_ADC_CHANNELS];
volatile uint16_t adc_readings[N_IR_ADC_CHANNELS];
uint8_t current_step = 0; // step 0: IR LED off, step 1: IR LED on
uint8_t current_dmux_index = 0;
const uint8_t dmux_channels[IR_SENSOR_COUNT] = {0u, 1u, 7u};
volatile uint32_t adc_frame_sequence = 0;

// Serial communication with ESP8266
uint8_t rxByte;
char rxBuffer[256];
uint8_t rxIndex = 0;
volatile uint8_t posReady = 0;
volatile uint8_t cmdReady  = 0;
char cmdBuffer[256];
char posBuffer[256];

// robot position
float robot_x = 0.0f;
float robot_y = 0.0f;
float robot_theta = 0.0f;
float robot_theta_error = 0.0f;

uint8_t initial_pos = 1;
volatile uint8_t broadcastPOS_due = 0; // flag to broadcast position
volatile uint8_t broadcastMAP_due = 0; // flag to broadcast position plus SCE maps
volatile uint8_t debug_due = 0; // periodic or obstacle-state-change debug message
volatile uint8_t odom_due = 0; // flag to request odom update
volatile uint32_t cam_due  = 0; // flag for request cam pos update

static camera_state_t camera_state = CAMERA_IDLE;
static uint32_t camera_state_started_ms = 0;
static uint16_t camera_saved_pwm_l = MOTOR_PWM_STOP;
static uint16_t camera_saved_pwm_r = MOTOR_PWM_STOP;
static char camera_saved_command = 'X';
static goto_state_t camera_saved_goto_state = GOTO_IDLE;

// Remote control
char command = 'X';
uint8_t new_cmd = 0;
float params[5];
uint32_t cmd_end;

// GOTO XY

volatile goto_state_t goto_state = GOTO_IDLE;
static uint32_t goto_rotate_started_ms = 0;
static uint32_t goto_backoff_until_ms = 0;
static uint32_t inactive_recheck_ms = 0;

float xt,yt; // target point (center of the target cell)
int xt_i, yt_i; // grid indices of the target cell

// SCE memory
float visits_map[ROWS][COLS] = {0};
float penalties_map[ROWS][COLS] = {0};
float obstacles_map[ROWS][COLS] = {0};
static int last_visit_r = -1;
static int last_visit_c = -1;
static uint32_t sce_rng_state = 0x9E3779B9u ^ ((uint32_t)MID * 0x85EBCA6Bu);

static obstacle_status_t obstacle_status = {
	.cell_r = {-1, -1, -1},
	.cell_c = {-1, -1, -1}
};
static uint8_t obstacle_hit_streak[IR_SENSOR_COUNT] = {0};
static uint8_t obstacle_clear_streak[IR_SENSOR_COUNT] = {0};
static int8_t obstacle_map_candidate_r[IR_SENSOR_COUNT] = {-1, -1, -1};
static int8_t obstacle_map_candidate_c[IR_SENSOR_COUNT] = {-1, -1, -1};
static uint32_t obstacle_map_candidate_since_ms[IR_SENSOR_COUNT] = {0};
static uint8_t obstacle_map_ready_mask = 0;

// inter swarm communication
float other_robots[MAX_OTHER_ROBOTS][2];
uint8_t other_robots_ids[MAX_OTHER_ROBOTS] = {0};
uint32_t other_robots_last_seen_ms[MAX_OTHER_ROBOTS] = {0};
uint8_t n_other_robots = 0;
static uint8_t accept_peer_maps = 0;
static char opos_queue[OPOS_QUEUE_DEPTH][OPOS_MESSAGE_SIZE];
static volatile uint8_t opos_q_head = 0;
static volatile uint8_t opos_q_tail = 0;
static volatile uint8_t opos_q_count = 0;
static volatile uint32_t opos_drop_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void Motors_Init(Motors *m,
		TIM_HandleTypeDef *tim_r, uint32_t ch_r,
		TIM_HandleTypeDef *tim_l, uint32_t ch_l);
void Motors_SetPWM(Motors *m, uint16_t pwm_l, uint16_t pwm_r);
void Motors_Stop(Motors *m);

/* --- Encoder / timers / callbacks --- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_odometry(void);

/* --- Camera position (UART request + LPF) --- */
static int  parse_pos_reply(const char *s, float *x, float *y, float *th);
static void lpf_update_pos(float x_meas, float y_meas, float th_meas);
static void camera_correction_step(uint8_t request_due);
static int camera_correction_busy(void);

/* --- DMUX + ADC scanning --- */
void Set_DMUX_Address(uint8_t address);
void ADC_Scan_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/* --- UART RX + command parsing --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void parse_command_if_ready(void);

/* --- SCE memory --- */
static inline int round_nearest(float v);
static int world_to_cell(float x, float y, int *r, int *c);
static uint32_t sce_random(void);

void  penalize_target_cell(void);
void  discount_penalties(void);
void  visits_map_update(float x, float y);
float fpow_simple(float base, unsigned exp);

/* --- Continuous obstacle sensing, classification and map projection --- */
static inline void unit_vec_from_theta(float th, float off, float *ux, float *uy);

static inline void obstacle_pos_from_pos_and_offset(float x, float y, float th,
		float dist_m, float off_rad,
		float *ox, float *oy);
static void obstacle_status_update(void);
static int obstacle_status_is_fresh(uint32_t now_ms);
static int mark_cached_static_obstacles(void);
static uint8_t obstacle_mask_blocking_motion(void);
static int remaining_path_has_static_obstacle(void);

static inline void cell_center(int r, int c, float *cx, float *cy);


/* --- GOTO XY helpers + state machine --- */
static inline int cell_is_free(int r, int c);
static inline int peer_is_fresh(int i, uint32_t now_ms);
static int active_peer_count(uint32_t now_ms);
static int cell_is_occupied_by_peer(int r, int c, uint32_t now_ms);
static int compute_reachable_cells(int sr, int sc,
		uint8_t reachable[ROWS][COLS], uint8_t block_peers,
		uint32_t now_ms);
static inline float h_manhattan(int r, int c, int tr, int tc);
static int astar_plan_cells(int sr, int sc, int tr, int tc);

static inline int near_known_robot(float ox, float oy, float gate_m);

static float wrap_pi(float a);

static float desired_theta_to_target(float x, float y, float th, float tx, float ty);
static float heading_error_to_target(float x, float y, float th, float tx, float ty);
static float dist_to_target(float x, float y, float tx, float ty);

void gotoXY(void);
static void experiment_reset(void);
void handle_command(void);

/* --- Inter-swarm / broadcasts --- */
void handle_opos_if_ready(void);
void broadcast_pos(uint8_t include_maps);

/* --- DEBUG --- */
void debug_send_state(void)
{
    // The 8x11 maps can produce roughly 1.5 KiB at maximum field widths.
    static char tx[2048];
    int len = 0;

	uint16_t adc0, adc1, adc2;
	uint8_t queue_count;
	uint32_t queue_drops;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
	queue_count = opos_q_count;
	queue_drops = opos_drop_count;
	__set_PRIMASK(primask);
	adc0 = obstacle_status.adc_mv[0];
	adc1 = obstacle_status.adc_mv[1];
	adc2 = obstacle_status.adc_mv[2];

	uint32_t now_ms = HAL_GetTick();
	uint32_t obstacle_age_ms = obstacle_status.frame_sequence
			? (now_ms - obstacle_status.updated_ms) : UINT32_MAX;

	// IR fields: frame, age_ms, fresh, raw, confirmed, robot, static, goto_state.
	// Mask bits: bit 0 = front, bit 1 = front-right, bit 2 = front-left.
	// IC contains the projected row/column for front, front-right, front-left.
	len += snprintf(tx + len, sizeof(tx) - len,
			"DEBUG#IR:%lu,%lu,%u,%X,%X,%X,%X,%u#IC:%d,%d;%d,%d;%d,%d"
			"#N:%c,%u,%u,%u,%d,%d,%.3f,%.3f,%.3f,%.3f#R:",
			(unsigned long)obstacle_status.frame_sequence,
			(unsigned long)obstacle_age_ms,
			(unsigned int)obstacle_status_is_fresh(now_ms),
			(unsigned int)obstacle_status.raw_mask,
			(unsigned int)obstacle_status.detected_mask,
			(unsigned int)obstacle_status.robot_mask,
			(unsigned int)obstacle_status.static_mask,
			(unsigned int)goto_state,
			(int)obstacle_status.cell_r[0], (int)obstacle_status.cell_c[0],
			(int)obstacle_status.cell_r[1], (int)obstacle_status.cell_c[1],
			(int)obstacle_status.cell_r[2], (int)obstacle_status.cell_c[2],
			command,
			(unsigned int)camera_state,
			(unsigned int)motors.pwm_l,
			(unsigned int)motors.pwm_r,
			path_idx, path_len,
			robot_x, robot_y, robot_theta, robot_theta_error);
	for (int i = 0; i < n_other_robots; i++)
	{
		uint32_t age_ms = now_ms - other_robots_last_seen_ms[i];
		len += snprintf(tx + len, sizeof(tx) - len,
				"%s%u,%.3f,%.3f,%lu",
				(i == 0) ? "" : ";",
				(unsigned int)other_robots_ids[i],
				other_robots[i][0], other_robots[i][1],
				(unsigned long)age_ms);
	}

	len += snprintf(tx + len, sizeof(tx) - len, "#Q:%u,%lu#V:",
			(unsigned int)queue_count, (unsigned long)queue_drops);

    for (int r = 0; r < ROWS; r++)
    {
        if (r > 0) len += snprintf(tx + len, sizeof(tx) - len, ";");

        for (int c = 0; c < COLS; c++)
        {
            uint16_t v = (uint16_t)(visits_map[r][c] + 0.5f);
            len += snprintf(tx + len, sizeof(tx) - len,
                            "%u%s", v, (c == COLS - 1) ? "" : ",");
        }
    }

    len += snprintf(tx + len, sizeof(tx) - len, "#P10:");

    for (int r = 0; r < ROWS; r++)
    {
        if (r > 0) len += snprintf(tx + len, sizeof(tx) - len, ";");

        for (int c = 0; c < COLS; c++)
        {
            uint16_t p = (uint16_t)(penalties_map[r][c] * 10.0f + 0.5f);
            len += snprintf(tx + len, sizeof(tx) - len,
                            "%u%s", p, (c == COLS - 1) ? "" : ",");
        }
    }

    len += snprintf(tx + len, sizeof(tx) - len, "#O:");

    for (int r = 0; r < ROWS; r++)
    {
        uint16_t mask = 0;

        for (int c = 0; c < COLS; c++)
        {
            if (obstacles_map[r][c] >= 1.0f)
                mask |= (1u << c);
        }

        len += snprintf(tx + len, sizeof(tx) - len,
                        "%03X%s", mask, (r == ROWS - 1) ? "" : ",");
    }

    len += snprintf(tx + len, sizeof(tx) - len,
                    "#A:%u,%u,%u\n",
                    adc0, adc1, adc2);

    if (len > 0 && len < (int)sizeof(tx))
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)tx, (uint16_t)len, 100);
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ------------------------------------------------------------ Controlling motors
void Motors_Init(Motors *m,
		TIM_HandleTypeDef *tim_r, uint32_t ch_r,
		TIM_HandleTypeDef *tim_l, uint32_t ch_l)
{
	m->tim_r = tim_r; m->ch_r = ch_r;
	m->tim_l = tim_l; m->ch_l = ch_l;

	m->pwm_l = MOTOR_PWM_STOP;
	m->pwm_r = MOTOR_PWM_STOP;
	Motors_Stop(m);
}

void Motors_SetPWM(Motors *m, uint16_t pwm_l, uint16_t pwm_r)
{
	m->pwm_l = pwm_l;
	m->pwm_r = pwm_r;

	__HAL_TIM_SET_COMPARE(m->tim_r, m->ch_r, pwm_r);
	__HAL_TIM_SET_COMPARE(m->tim_l, m->ch_l, pwm_l);
}

void Motors_Stop(Motors *m)
{
	Motors_SetPWM(m, MOTOR_PWM_STOP, MOTOR_PWM_STOP);
}

// External interrupt starts detection
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_7)
	{
		// PA7 triggered - start TIM2 for debouncing
		__HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset counter to 0
		HAL_TIM_Base_Start_IT(&htim2);      // Start timer with interrupt
	}
	else if(GPIO_Pin == GPIO_PIN_5)
	{
		// PA5 triggered - start TIM5 for debouncing
		__HAL_TIM_SET_COUNTER(&htim5, 0);  // Reset counter to 0
		HAL_TIM_Base_Start_IT(&htim5);      // Start timer with interrupt
	}
}

// Validation of the interrupt by a timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		// TIM2 expired - check if PA7 is still HIGH
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
		{
			// Valid signal, increment counter
			uint32_t cms = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1); // current motor speed
			if (cms < MOTOR_PWM_STOP) // motor right rotates forward
				encoder_right_count++;
			else
				encoder_right_count--;
		}
		HAL_TIM_Base_Stop_IT(&htim2);  // Stop timer
	}
	else if(htim == &htim5)
	{
		// TIM5 expired - check if PA5 is still HIGH
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
		{
			// Valid signal, increment counter
			uint32_t cms = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1); // current motor speed
			if (cms > MOTOR_PWM_STOP) // motor left rotates forward (backward, but physically reversed)
				encoder_left_count++;
			else
				encoder_left_count--;
		}
		HAL_TIM_Base_Stop_IT(&htim5);  // Stop timer
	}
	else if(htim == &htim9)
	{
		// Stop timer
		HAL_TIM_Base_Stop_IT(&htim9);

		// Restart ADC+DMA for next reading
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, IRD_NUM_SAMPLES);
	}
	if (htim == &htim10)
	{
		// request update odometry every 10ms (100Hz) [done in the main loop]
		odom_due++;

		static uint16_t div200 = 0;
		if (++div200 >= 200) {     // 2 seconds @ 100 Hz
			div200 = 0;
			cam_due++;
		}

		static uint16_t div50 = 0;
		if (++div50 >= 50) {       // position at 2 Hz
			div50 = 0;
			broadcastPOS_due = 1;
		}

		static uint16_t div100 = 0;
		if (++div100 >= 100) {     // full SCE maps at 1 Hz
			div100 = 0;
			broadcastMAP_due = 1;
			debug_due = 1;
		}
	}
}

// Odometry calculations
void update_odometry(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	int32_t curr_left  = encoder_left_count;
	int32_t curr_right = encoder_right_count;
	__set_PRIMASK(primask);

	static int32_t prev_left = 0;
	static int32_t prev_right = 0;

	// Encoder deltas
	int32_t delta_left_counts  = curr_left - prev_left;
	int32_t delta_right_counts = curr_right - prev_right;

	prev_left  = curr_left;
	prev_right = curr_right;

	// Convert to distance [m]
	float distance_left  = (2.0f * M_PI * WHEEL_RADIUS_M / ENCODER_RES) * delta_left_counts;
	float distance_right = (2.0f * M_PI * WHEEL_RADIUS_M / ENCODER_RES) * delta_right_counts;

	// Compute linear and angular displacement
	float delta_s = (distance_right + distance_left) / 2.0f;
	float delta_theta = (distance_right - distance_left) / WHEEL_BASE_M;

	// Update robot pos
	robot_x += delta_s * cosf(robot_theta + delta_theta / 2.0f + M_PI / 2.0f);
	robot_y += delta_s * sinf(robot_theta + delta_theta / 2.0f + M_PI / 2.0f);
	robot_theta += delta_theta;

	// Keep theta within -π ... π
	if (robot_theta > M_PI)
		robot_theta -= 2.0f * M_PI;
	else if (robot_theta < -M_PI)
		robot_theta += 2.0f * M_PI;
}

// cam pos
static int parse_pos_reply(const char *s, float *x, float *y, float *th)
{
	// very strict format; returns 1 on success
	// "POS %f %f %f"
	return (sscanf(s, "POS#%f#%f#%f", x, y, th) == 3) ? 1 : 0;
}

// low pass filter pos update from camera pos
static void lpf_update_pos(float x_meas, float y_meas, float th_meas)
{
	// onlt the very first cam pos must be used as an absoloute update (reset state)
	if (initial_pos)
	{
		robot_x = x_meas;
		robot_y = y_meas;
		robot_theta = th_meas;
		initial_pos = 0;
	}
	else
	{
		// LPF for x,y
		robot_x = (1.0f - POS_LPF_ALPHA) * robot_x + POS_LPF_ALPHA * x_meas;
		robot_y = (1.0f - POS_LPF_ALPHA) * robot_y + POS_LPF_ALPHA * y_meas;

		// LPF for angle with wrap handling (use shortest angle difference)
		float d = th_meas - robot_theta;
		while (d >  M_PI) d -= 2.0f * M_PI;
		while (d < -M_PI) d += 2.0f * M_PI;

		robot_theta += POS_LPF_ALPHA * d;
	}

	// normalize robot_theta * even for abs update
	if (robot_theta >  M_PI) robot_theta -= 2.0f * M_PI;
	if (robot_theta < -M_PI) robot_theta += 2.0f * M_PI;

	// Do not wait for the next 2 Hz slot to share a camera-corrected pose.
	broadcastPOS_due = 1;
}

// Non-blocking camera correction. Keeping this state machine out of a wait loop
// lets IR classification, peer RX and periodic pose TX continue during the
// mechanical settling and ROS response windows.
static void camera_correction_step(uint8_t request_due)
{
	uint32_t now_ms = HAL_GetTick();

	if (camera_state == CAMERA_IDLE)
	{
		if (!request_due || command == 'P')
			return;

		camera_saved_pwm_l = motors.pwm_l;
		camera_saved_pwm_r = motors.pwm_r;
		camera_saved_command = command;
		camera_saved_goto_state = goto_state;
		Motors_Stop(&motors);
		camera_state_started_ms = now_ms;
		camera_state = CAMERA_SETTLING;
		return;
	}

	if (camera_state == CAMERA_SETTLING)
	{
		if ((now_ms - camera_state_started_ms) < STOP_SETTLE_MS)
			return;

		// Discard any late reply from an older request before starting a new one.
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		posReady = 0;
		posBuffer[0] = '\0';
		__set_PRIMASK(primask);

		const char req[] = "POS?\n";
		(void)HAL_UART_Transmit(&huart1, (uint8_t*)req,
				(uint16_t)(sizeof(req) - 1), 50);
		camera_state_started_ms = HAL_GetTick();
		camera_state = CAMERA_WAITING;
		return;
	}

	int finished = 0;
	if (posReady)
	{
		char local[256];
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		posReady = 0;
		strncpy(local, posBuffer, sizeof(local));
		local[sizeof(local) - 1] = '\0';
		__set_PRIMASK(primask);

		float x_meas, y_meas, th_meas;
		if (parse_pos_reply(local, &x_meas, &y_meas, &th_meas))
			lpf_update_pos(x_meas, y_meas, th_meas);
		finished = 1;
	}
	else if ((now_ms - camera_state_started_ms) >= POS_WAIT_MS)
	{
		finished = 1;
	}

	if (finished)
	{
		camera_state = CAMERA_IDLE;
		// A peer map or a newly parsed command may have cancelled the route while
		// the camera request was in flight. Never resume stale motor commands.
		if (command == camera_saved_command && goto_state == camera_saved_goto_state)
			Motors_SetPWM(&motors, camera_saved_pwm_l, camera_saved_pwm_r);
		else
			Motors_Stop(&motors);
	}
}

static int camera_correction_busy(void)
{
	return (camera_state != CAMERA_IDLE);
}

// Function to set DMUX address
void Set_DMUX_Address(uint8_t address)
{
	// address is 0-7 (3 bits)
	HAL_GPIO_WritePin(DMUX_PORT, DMUX_A_PIN, (address & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DMUX_PORT, DMUX_B_PIN, (address & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DMUX_PORT, DMUX_C_PIN, (address & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ADC_Scan_Init(void)
{
	Set_DMUX_Address(dmux_channels[0]);  // Start with first channel (0)
	HAL_GPIO_WritePin(DMUX_PORT, DMUX_EN, GPIO_PIN_RESET); // Disable DMUX
	HAL_Delay(1);  // Small delay for DMUX to settle
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, IRD_NUM_SAMPLES);
}

// DMA Complete Callback - Process data and start timer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		// Stop DMA
		HAL_ADC_Stop_DMA(&hadc1);

		// Calculate average
//		uint32_t sum = 0;
//		for(int i = 0; i < IRD_NUM_SAMPLES; i++) {
//			sum += adc_buffer[i];
//		}
//		float miliVolts = (sum * 3300.0f) / (IRD_NUM_SAMPLES * 4095.0f);

		// Median
		uint16_t samples[IRD_NUM_SAMPLES];

		for (int i = 0; i < IRD_NUM_SAMPLES; i++)
		    samples[i] = adc_buffer[i];

		for (int i = 1; i < IRD_NUM_SAMPLES; i++) {
		    uint16_t key = samples[i];
		    int j = i - 1;

		    while (j >= 0 && samples[j] > key) {
		        samples[j + 1] = samples[j];
		        j--;
		    }

		    samples[j + 1] = key;
		}

		uint16_t adc_median = samples[IRD_NUM_SAMPLES / 2];

		float miliVolts = adc_median * 3300.0f / 4095.0f;
		uint8_t channel = dmux_channels[current_dmux_index];

		// Store the reading based on current step
		if(current_step == 0)  // Step 0: DMUX disabled (OFF reading)
		{
			adc_readings_off[channel] = (uint16_t)miliVolts;
			current_step = 1;

			// Enable DMUX for ON reading
			HAL_GPIO_WritePin(DMUX_PORT, DMUX_EN, GPIO_PIN_SET);
		}
		else  // Step 1: DMUX enabled (ON reading)
		{
			adc_readings_on[channel] = (uint16_t)miliVolts;

			// Calculate difference
			if (adc_readings_off[channel] < adc_readings_on[channel])
				adc_readings[channel] = adc_readings_on[channel] - adc_readings_off[channel];
			else
				adc_readings[channel] = 0; // invalid reading

			current_step = 0;

			// Disable DMUX
			HAL_GPIO_WritePin(DMUX_PORT, DMUX_EN, GPIO_PIN_RESET);

			// Move to next channel
			current_dmux_index++;
			if(current_dmux_index >= IR_SENSOR_COUNT) {
				current_dmux_index = 0;
				// Publish one coherent frame after all channels are complete.
				adc_frame_sequence++;
			}

			// Set new DMUX address for next channel
			Set_DMUX_Address(dmux_channels[current_dmux_index]);
		}

		// Start timer for settling delay
		__HAL_TIM_SET_COUNTER(&htim9, 0);  // Reset counter
		HAL_TIM_Base_Start_IT(&htim9);     // Start timer with interrupt
	}
}

// ------------------------------------------------------------ Serial connection to the Wifi cheap
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1)
	{
		// ESP8266 Serial.println() terminates forwarded OPOS packets with CRLF.
		// Ignore CR so protocol tokens (especially P1 and the final M1 row mask)
		// do not retain a trailing '\r' and fail strict parsing.
		if (rxByte == '\r')
		{
			// Ignore it; reception is re-armed below.
		}
		else if (rxByte == '\n')
		{
			rxBuffer[rxIndex] = '\0';

			// Classify message by prefix
			if (strncmp(rxBuffer, "CMD#", 4) == 0)
			{
				// Only copy if slot is free (avoid overwriting an unprocessed command)
				if (!cmdReady)
				{
					strncpy(cmdBuffer, rxBuffer, sizeof(cmdBuffer));
					cmdBuffer[sizeof(cmdBuffer) - 1] = '\0';
					cmdReady = 1;
				}
				// else drop it
			}
			else if (strncmp(rxBuffer, "POS#", 4) == 0)
			{
				if (!posReady)
				{
					strncpy(posBuffer, rxBuffer, sizeof(posBuffer));
					posBuffer[sizeof(posBuffer) - 1] = '\0';
					posReady = 1;
				}
			}
			else if (strncmp(rxBuffer, "OPOS#", 5) == 0)
			{
				if (opos_q_count < OPOS_QUEUE_DEPTH)
				{
					uint8_t slot = opos_q_head;
					strncpy(opos_queue[slot], rxBuffer, sizeof(opos_queue[slot]));
					opos_queue[slot][sizeof(opos_queue[slot]) - 1] = '\0';
					opos_q_head = (uint8_t)((slot + 1u) % OPOS_QUEUE_DEPTH);
					opos_q_count++;
				}
				else
				{
					int stored = 0;

					// Full M1 snapshots are cumulative and more important than the
					// high-rate P1 poses. If necessary, evict a queued pose so the
					// coverage/obstacle history is never dropped behind pose traffic.
					if (strstr(rxBuffer, "#M1#") != NULL)
					{
						uint8_t slot = opos_q_tail;
						for (uint8_t i = 0; i < opos_q_count; i++)
						{
							if (strstr(opos_queue[slot], "#P1") != NULL)
							{
								strncpy(opos_queue[slot], rxBuffer, sizeof(opos_queue[slot]));
								opos_queue[slot][sizeof(opos_queue[slot]) - 1] = '\0';
								stored = 1;
								break;
							}
							slot = (uint8_t)((slot + 1u) % OPOS_QUEUE_DEPTH);
						}
					}

					opos_drop_count++;
					(void)stored;
				}
			}
			rxIndex = 0;
		}
		else if (rxIndex < (sizeof(rxBuffer) - 1))
		{
			rxBuffer[rxIndex++] = rxByte;
		}
		else
		{
			// Overflow: reset line
			rxIndex = 0;
		}

		HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	}
}

// checks flag cmdReady
// copies cmdBuffer tp local
// sets flag new_cmd
// sets command var and params array
void parse_command_if_ready(void)
{
	if (!cmdReady) return;

	char local[64];

	// ---- atomic snapshot of command buffer ----
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		cmdReady = 0;
		strncpy(local, cmdBuffer, sizeof(local));
		local[sizeof(local) - 1] = '\0';
		__set_PRIMASK(primask);
	}

	// ---- parse cmd ----
	char *s = local + 4;                 // points to "<cmd>#..."
	if (*s == '\0') return;
	new_cmd = 1;                         // mark it as a new command

	command = *s++;
	if (*s != '#') return;               // must be "CMD#<cmd>#"
	s++;                                 // now points to first param (or '\0')

	// ---- parse up to 5 params (as double -> float) ----
	int   nparams = 0;

	while (nparams < 5 && *s != '\0')
	{
		// find token end
		char *end = strchr(s, '#');
		if (end) *end = '\0';            // temporarily terminate this token

		// skip empty tokens
		while (isspace((unsigned char)*s)) s++;

		if (*s != '\0')
		{
			// strtod handles both ints and floats (and scientific notation)
			char *conv_end = NULL;
			double val = strtod(s, &conv_end);

			// accept only if token is a valid number (allow trailing spaces)
			while (conv_end && isspace((unsigned char)*conv_end)) conv_end++;

			if (conv_end && *conv_end == '\0')
			{
				params[nparams++] = (float)val;
			}
			else
			{
				// invalid number token -> stop parsing further params
				break;
			}
		}

		if (!end) break;                 // no more tokens
		s = end + 1;                     // next token start
	}
}

static inline int round_nearest(float v)
{
	return (v >= 0.0f) ? (int)(v + 0.5f) : (int)(v - 0.5f);
}

static int world_to_cell(float x, float y, int *r, int *c)
{
	int local_c = round_nearest((x - X0) / CELL);
	int local_r = round_nearest((y - Y0) / CELL);

	if (local_r < 0 || local_r >= ROWS || local_c < 0 || local_c >= COLS)
		return 0;

	*r = local_r;
	*c = local_c;
	return 1;
}

static uint32_t sce_random(void)
{
	// Small deterministic PRNG with a robot-specific seed for SCE random tie breaks.
	uint32_t x = sce_rng_state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	sce_rng_state = x;
	return x;
}

void penalize_target_cell(void)
{
	if (penalties_map[yt_i][xt_i] < MAX_VISIT_AND_PENALTY_COUNT)
		penalties_map[yt_i][xt_i] += 1;
}

void discount_penalties(void)
{
	for (uint32_t i = 0; i < ROWS; i++) {
		for (uint32_t j = 0; j < COLS; j++) {
			penalties_map[i][j] *= SCE_PENALTY_DECAY;
		}
	}
}

void visits_map_update(float x, float y)
{
	int r, c;
	if (!world_to_cell(x, y, &r, &c))
		return;

	// A robot physically occupying a cell is stronger evidence than an IR-based
	// obstacle estimate. Keep visited and obstacle mutually exclusive.
	int obstacle_cleared = (obstacles_map[r][c] >= 1.0f);
	if (obstacle_cleared)
		obstacles_map[r][c] = 0.0f;

	// A visit is a cell-entry event, not one increment per main-loop iteration.
	if (r == last_visit_r && c == last_visit_c)
	{
		if (obstacle_cleared)
		{
			broadcastPOS_due = 1;
			broadcastMAP_due = 1;
			debug_due = 1;
		}
		return;
	}

	last_visit_r = r;
	last_visit_c = c;
	discount_penalties();
	// On the physical grid, a confirmed covered cell must remain strongly
	// unattractive; a value of 1 was too weak against the distance term.
	visits_map[r][c] = MAX_VISIT_AND_PENALTY_COUNT;

	// Algorithm 1 communicates after every successful cell move.
	broadcastPOS_due = 1;
	broadcastMAP_due = 1;
}

float fpow_simple(float base, unsigned exp)
{
	float r = 1.0f;
	while (exp--) r *= base;
	return r;
}

static inline void unit_vec_from_theta(float th, float off, float *ux, float *uy)
{
	// robot forward is (theta + pi/2). Add sensor offset around that.
	float a = th + (float)M_PI_2 + off;
	*ux = cosf(a);
	*uy = sinf(a);
}

static inline void obstacle_pos_from_pos_and_offset(float x, float y, float th,
		float dist_m, float off_rad,
		float *ox, float *oy)
{
	float ux, uy;
	unit_vec_from_theta(th, off_rad, &ux, &uy);
	*ox = x + dist_m * ux;
	*oy = y + dist_m * uy;
}

static inline void cell_center(int r, int c, float *cx, float *cy)
{
    *cx = c * CELL + X0;
    *cy = r * CELL + Y0;
}

static inline int cell_is_free(int r, int c)
{
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS) return 0;
    return (obstacles_map[r][c] < 1.0f);
}

static inline int peer_is_fresh(int i, uint32_t now_ms)
{
	return (i >= 0 && i < n_other_robots &&
			(now_ms - other_robots_last_seen_ms[i]) <= PEER_POSE_TIMEOUT_MS);
}

static int active_peer_count(uint32_t now_ms)
{
	int count = 0;
	for (int i = 0; i < n_other_robots; i++)
		if (peer_is_fresh(i, now_ms)) count++;
	return count;
}

static int cell_is_occupied_by_peer(int r, int c, uint32_t now_ms)
{
	for (int i = 0; i < n_other_robots; i++)
	{
		int peer_r, peer_c;
		if (peer_is_fresh(i, now_ms) &&
				world_to_cell(other_robots[i][0], other_robots[i][1], &peer_r, &peer_c) &&
				peer_r == r && peer_c == c)
			return 1;
	}
	return 0;
}

static inline float h_manhattan(int r, int c, int tr, int tc)
{
    int dr = (r > tr) ? (r - tr) : (tr - r);
    int dc = (c > tc) ? (c - tc) : (tc - c);
    return (float)(dr + dc);
}

static int compute_reachable_cells(int sr, int sc,
		uint8_t reachable[ROWS][COLS], uint8_t block_peers,
		uint32_t now_ms)
{
	int queue_r[ROWS * COLS];
	int queue_c[ROWS * COLS];
	int head = 0;
	int tail = 0;
	static const int dr4[4] = { -1, +1, 0, 0 };
	static const int dc4[4] = { 0, 0, -1, +1 };

	memset(reachable, 0, ROWS * COLS * sizeof(reachable[0][0]));
	if (sr < 0 || sr >= ROWS || sc < 0 || sc >= COLS)
		return 0;

	// The robot's current cell is always a valid flood-fill origin. A noisy IR
	// projection can occasionally mark that cell while the robot is still in it.
	reachable[sr][sc] = 1;
	queue_r[tail] = sr;
	queue_c[tail] = sc;
	tail++;

	while (head < tail)
	{
		int cr = queue_r[head];
		int cc = queue_c[head];
		head++;

		for (int k = 0; k < 4; k++)
		{
			int nr = cr + dr4[k];
			int nc = cc + dc4[k];
			if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS)
				continue;
			if (reachable[nr][nc] || !cell_is_free(nr, nc) ||
					(block_peers && cell_is_occupied_by_peer(nr, nc, now_ms)))
				continue;

			reachable[nr][nc] = 1;
			queue_r[tail] = nr;
			queue_c[tail] = nc;
			tail++;
		}
	}

	return tail;
}

static int astar_plan_cells(int sr, int sc, int tr, int tc)
{
	uint32_t now_ms = HAL_GetTick();

    // arrays
    static uint8_t open[ROWS][COLS];
    static uint8_t closed[ROWS][COLS];
    static float   g[ROWS][COLS];
    static float   f[ROWS][COLS];
    static int16_t pr[ROWS][COLS];
    static int16_t pc[ROWS][COLS];

    // init
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            open[r][c] = 0;
            closed[r][c] = 0;
            g[r][c] = 1e9f;
            f[r][c] = 1e9f;
            pr[r][c] = -1;
            pc[r][c] = -1;
        }
    }

	if (sr < 0 || sr >= ROWS || sc < 0 || sc >= COLS) return 0;
    if (!cell_is_free(tr, tc)) return 0;
    if (cell_is_occupied_by_peer(tr, tc, now_ms)) return 0;

    open[sr][sc] = 1;
    g[sr][sc] = 0.0f;
    f[sr][sc] = h_manhattan(sr, sc, tr, tc);

    // A*
    while (1)
    {
        // find best open node
        int cr = -1, cc = -1;
        float best_f = 1e9f;
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                if (open[r][c] && f[r][c] < best_f) {
                    best_f = f[r][c];
                    cr = r; cc = c;
                }
            }
        }
        if (cr < 0) return 0;              // no path
        if (cr == tr && cc == tc) break;   // reached

        open[cr][cc] = 0;
        closed[cr][cc] = 1;

        // 4-neighbors
        static const int dr4[4] = { -1, +1,  0,  0 };
        static const int dc4[4] = {  0,  0, -1, +1 };

        for (int k = 0; k < 4; k++)
        {
            int nr = cr + dr4[k];
            int nc = cc + dc4[k];

            if (!cell_is_free(nr, nc)) continue;
            if (cell_is_occupied_by_peer(nr, nc, now_ms)) continue;
            if (closed[nr][nc]) continue;

            float tentative_g = g[cr][cc] + 1.0f; // uniform cost

            if (!open[nr][nc] || tentative_g < g[nr][nc])
            {
                pr[nr][nc] = (int16_t)cr;
                pc[nr][nc] = (int16_t)cc;
                g[nr][nc] = tentative_g;
                f[nr][nc] = tentative_g + h_manhattan(nr, nc, tr, tc);
                open[nr][nc] = 1;
            }
        }
    }

    // reconstruct path into path_r/path_c (reverse, then flip)
    int rr = tr, cc = tc;
    int tmp_r[ROWS*COLS];
    int tmp_c[ROWS*COLS];
    int tmp_len = 0;

    while (!(rr == sr && cc == sc))
    {
        if (tmp_len >= ROWS*COLS) return 0;
        tmp_r[tmp_len] = rr;
        tmp_c[tmp_len] = cc;
        tmp_len++;

        int16_t ppr = pr[rr][cc];
        int16_t ppc = pc[rr][cc];
        if (ppr < 0 || ppc < 0) return 0; // should not happen
        rr = ppr;
        cc = ppc;
    }

    // include start? we don't need it as waypoint, so we flip only the moves
    path_len = tmp_len;
    path_idx = 0;
    for (int i = 0; i < tmp_len; i++) {
        path_r[i] = tmp_r[tmp_len - 1 - i];
        path_c[i] = tmp_c[tmp_len - 1 - i];
    }

    return 1;
}

static inline int near_known_robot(float ox, float oy, float gate_m)
{
	uint32_t now_ms = HAL_GetTick();
    float gate2 = gate_m * gate_m;
    for (int i = 0; i < n_other_robots; i++) {
		if (!peer_is_fresh(i, now_ms)) continue;
        float dx = other_robots[i][0] - ox;
        float dy = other_robots[i][1] - oy;
        if ((dx*dx + dy*dy) <= gate2) return 1;
    }
    return 0;
}

// Consume each completed ADC scan once and keep a continuously available,
// debounced interpretation of the three forward-facing IR sensors.
static void obstacle_status_update(void)
{
	static uint32_t last_processed_sequence = 0;
	static const uint16_t threshold_mv[IR_SENSOR_COUNT] = {
		OBSTACLE_TH0_MV, OBSTACLE_TH1_MV, OBSTACLE_TH2_MV
	};
	static const uint16_t clear_threshold_mv[IR_SENSOR_COUNT] = {
		OBSTACLE_CLEAR_TH0_MV, OBSTACLE_CLEAR_TH1_MV, OBSTACLE_CLEAR_TH2_MV
	};
	static const float sensor_offset_rad[IR_SENSOR_COUNT] = {
		S0_OFF_RAD, S1_OFF_RAD, S2_OFF_RAD
	};

	uint32_t sequence;
	uint16_t adc_mv[IR_SENSOR_COUNT];
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	sequence = adc_frame_sequence;
	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
		adc_mv[i] = adc_readings[dmux_channels[i]];
	__set_PRIMASK(primask);

	if (sequence == 0 || sequence == last_processed_sequence)
		return;
	last_processed_sequence = sequence;

	float x, y, th;
	primask = __get_PRIMASK();
	__disable_irq();
	x = robot_x;
	y = robot_y;
	th = robot_theta;
	__set_PRIMASK(primask);

	uint8_t previous_detected = obstacle_status.detected_mask;
	uint8_t previous_robot = obstacle_status.robot_mask;
	uint8_t previous_static = obstacle_status.static_mask;
	uint8_t previous_map_ready = obstacle_map_ready_mask;
	uint8_t raw_mask = 0;
	uint8_t detected_mask = obstacle_status.detected_mask;

	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		obstacle_status.adc_mv[i] = adc_mv[i];

		if (adc_mv[i] > threshold_mv[i])
			raw_mask |= bit;

		if (detected_mask & bit)
		{
			// Once asserted, retain the detection through the hysteresis band and
			// clear it only after several readings below the lower threshold.
			obstacle_hit_streak[i] = 0;
			if (adc_mv[i] < clear_threshold_mv[i])
			{
				if (obstacle_clear_streak[i] < OBSTACLE_CONFIRM_FRAMES)
					obstacle_clear_streak[i]++;
			}
			else
			{
				obstacle_clear_streak[i] = 0;
			}

			if (obstacle_clear_streak[i] >= OBSTACLE_CONFIRM_FRAMES)
				detected_mask &= (uint8_t)~bit;
		}
		else
		{
			obstacle_clear_streak[i] = 0;
			if (adc_mv[i] > threshold_mv[i])
			{
				if (obstacle_hit_streak[i] < OBSTACLE_CONFIRM_FRAMES)
					obstacle_hit_streak[i]++;
			}
			else
			{
				obstacle_hit_streak[i] = 0;
			}

			if (obstacle_hit_streak[i] >= OBSTACLE_CONFIRM_FRAMES)
				detected_mask |= bit;
		}
	}

	obstacle_status.raw_mask = raw_mask;
	obstacle_status.detected_mask = detected_mask;
	obstacle_status.robot_mask = 0;
	obstacle_status.static_mask = 0;

	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		obstacle_status.cell_r[i] = -1;
		obstacle_status.cell_c[i] = -1;
		obstacle_status.hit_x[i] = INVALID_POS;
		obstacle_status.hit_y[i] = INVALID_POS;

		if (!(detected_mask & bit))
			continue;

		float ox, oy;
		obstacle_pos_from_pos_and_offset(x, y, th, OBSTACLE_DIST_M,
				sensor_offset_rad[i], &ox, &oy);
		obstacle_status.hit_x[i] = ox;
		obstacle_status.hit_y[i] = oy;

		int r, c;
		if (world_to_cell(ox, oy, &r, &c))
		{
			obstacle_status.cell_r[i] = (int8_t)r;
			obstacle_status.cell_c[i] = (int8_t)c;
		}

		if (near_known_robot(ox, oy, ROBOT_AS_OBS_GATE_M))
			obstacle_status.robot_mask |= bit;
		else
			obstacle_status.static_mask |= bit;
	}

	uint32_t now_ms = HAL_GetTick();
	obstacle_status.frame_sequence = sequence;
	obstacle_status.updated_ms = now_ms;

	// Collision avoidance keeps the fast sensor debounce above, but persistent
	// mapping is deliberately slower. A candidate must remain classified as
	// static in the same projected grid cell while driving for the full hold time.
	// Any sensor clear, peer classification, cell change, or non-driving state
	// cancels that candidate before it can contaminate the shared map.
	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		int r = obstacle_status.cell_r[i];
		int c = obstacle_status.cell_c[i];
		int valid_static_cell =
				(goto_state == GOTO_DRIVE || goto_state == GOTO_BACKOFF) &&
				(obstacle_status.static_mask & bit) != 0 &&
				r >= 0 && r < ROWS && c >= 0 && c < COLS;

		if (!valid_static_cell)
		{
			obstacle_map_candidate_r[i] = -1;
			obstacle_map_candidate_c[i] = -1;
			obstacle_map_candidate_since_ms[i] = 0;
			obstacle_map_ready_mask &= (uint8_t)~bit;
			continue;
		}

		if (obstacle_map_candidate_r[i] != r ||
				obstacle_map_candidate_c[i] != c)
		{
			obstacle_map_candidate_r[i] = (int8_t)r;
			obstacle_map_candidate_c[i] = (int8_t)c;
			obstacle_map_candidate_since_ms[i] = now_ms;
			obstacle_map_ready_mask &= (uint8_t)~bit;
		}
		else if ((now_ms - obstacle_map_candidate_since_ms[i]) >=
				OBSTACLE_MAP_CONFIRM_MS)
		{
			obstacle_map_ready_mask |= bit;
		}
	}

	if (previous_detected != obstacle_status.detected_mask ||
			previous_robot != obstacle_status.robot_mask ||
			previous_static != obstacle_status.static_mask ||
			previous_map_ready != obstacle_map_ready_mask)
		debug_due = 1;
}

static int obstacle_status_is_fresh(uint32_t now_ms)
{
	return (obstacle_status.frame_sequence != 0 &&
			(now_ms - obstacle_status.updated_ms) <= OBSTACLE_STATUS_STALE_MS);
}

// Mapping is deliberately separate from continuous classification: sensing can
// run while idle or rotating, while only a static sensor/cell candidate held for
// OBSTACLE_MAP_CONFIRM_MS during driving/backoff reaches the persistent map.
static int mark_cached_static_obstacles(void)
{
	int current_r = -1;
	int current_c = -1;
	(void)world_to_cell(robot_x, robot_y, &current_r, &current_c);

	int changed = 0;
	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		if (!(obstacle_map_ready_mask & bit))
			continue;

		int r = obstacle_map_candidate_r[i];
		int c = obstacle_map_candidate_c[i];
		if (r < 0 || r >= ROWS || c < 0 || c >= COLS ||
				(r == current_r && c == current_c) ||
				visits_map[r][c] >= 1.0f)
			continue;

		if (obstacles_map[r][c] < 1.0f)
		{
			obstacles_map[r][c] = 1.0f;
			changed = 1;
		}
	}

	if (changed)
	{
		broadcastPOS_due = 1;
		broadcastMAP_due = 1;
		debug_due = 1;
	}
	return changed;
}

// A front hit is an immediate translation hazard. Angled sensors block motion
// only when their projected cell is the next waypoint; otherwise they can see
// an obstacle beside the robot without preventing a safe turn or forward path.
static uint8_t obstacle_mask_blocking_motion(void)
{
	uint8_t detected = obstacle_status.detected_mask;
	uint8_t blocking = detected & 0x01u;

	if (path_idx < 0 || path_idx >= path_len)
		return blocking;

	int next_r = path_r[path_idx];
	int next_c = path_c[path_idx];
	for (uint32_t i = 1; i < IR_SENSOR_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		if ((detected & bit) &&
				obstacle_status.cell_r[i] == next_r &&
				obstacle_status.cell_c[i] == next_c)
			blocking |= bit;
	}
	return blocking;
}

static int remaining_path_has_static_obstacle(void)
{
	for (int i = path_idx; i < path_len; i++)
	{
		int r = path_r[i];
		int c = path_c[i];
		if (r >= 0 && r < ROWS && c >= 0 && c < COLS &&
				obstacles_map[r][c] >= 1.0f)
			return 1;
	}
	return 0;
}

static float wrap_pi(float a)
{
	while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
	while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
	return a;
}

static float desired_theta_to_target(float x, float y, float th, float tx, float ty)
{
	(void)x; (void)y; (void)th;
	float dx = tx - x;
	float dy = ty - y;
	float phi = atan2f(dy, dx);              // standard world bearing
	float th_des = phi - (float)M_PI_2;      // convert to your theta convention
	return wrap_pi(th_des);
}

static float heading_error_to_target(float x, float y, float th, float tx, float ty)
{
	float th_des = desired_theta_to_target(x, y, th, tx, ty);
	return wrap_pi(th - th_des);
}

static float dist_to_target(float x, float y, float tx, float ty)
{
	float dx = tx - x;
	float dy = ty - y;
	return sqrtf(dx*dx + dy*dy);
}

// non-blocking step function
void gotoXY()
{
	uint32_t now_ms = HAL_GetTick();

	// DONE/INACTIVE mean there is no active route. Never consume stale waypoints
	// in these states; doing so allowed
	// path_idx to increment once per main-loop pass and eventually overflow.
	if (goto_state == GOTO_IDLE || goto_state == GOTO_DONE ||
			goto_state == GOTO_INACTIVE)
		return;

	if (goto_state == GOTO_BACKOFF)
	{
		// A blocking hit changes state immediately, so finish the static
		// confirmation during backoff and commit it once the hold time elapses.
		if (obstacle_status_is_fresh(now_ms))
			(void)mark_cached_static_obstacles();

		if ((int32_t)(now_ms - goto_backoff_until_ms) < 0)
		{
			Motors_SetPWM(&motors, MOTOR_PWM_MAX_BACKWARD, MOTOR_PWM_MAX_FORWARD);
		}
		else
		{
			Motors_Stop(&motors);
			goto_state = GOTO_DONE;
			debug_due = 1;
		}
		return;
	}

	// snapshot pos atomically
	float x, y, th;
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		x  = robot_x;
		y  = robot_y;
		th = robot_theta;
		__set_PRIMASK(primask);
	}

	float d = dist_to_target(x, y, xt, yt);
	float e = heading_error_to_target(x, y, th, xt, yt);  // + => need CW, - => need CCW
	robot_theta_error = e;

	// stop condition
	if (d <= GOTO_DIST_OK_M)
	{
		Motors_Stop(&motors);

		path_idx++;
		if (path_idx >= path_len) {
		    goto_state = GOTO_DONE;
		} else {
		    cell_center(path_r[path_idx], path_c[path_idx], &xt, &yt);
		    goto_rotate_started_ms = now_ms;
		    goto_state = GOTO_ROTATE;
		}
		return;
	}

	switch (goto_state)
	{
	case GOTO_ROTATE:
	{
		// Alignment takes priority over the timeout. The camera may report the
		// final corrected heading at the timeout boundary; rejecting an already
		// aligned robot here caused repeated DONE/replan cycles without driving.
		if (fabsf(e) <= GOTO_THETA_OK_RAD)
		{
			Motors_Stop(&motors);
			goto_state = GOTO_DRIVE;
			debug_due = 1;
		}
		else if ((now_ms - goto_rotate_started_ms) > GOTO_ROTATE_TIMEOUT_MS)
		{
			Motors_Stop(&motors);
			penalize_target_cell();
			goto_state = GOTO_DONE;
			debug_due = 1;
			return;
		}
		else
		{
			float effort = ROT_PWM_MIN_DELTA + ROT_PWM_PER_RAD * fabsf(e);
			if (effort > ROT_PWM_MAX_DELTA) effort = ROT_PWM_MAX_DELTA;
			uint16_t delta = (uint16_t)effort;

			// rotate towards target
			if (e > 0.0f)
			{
				// CW
				Motors_SetPWM(&motors, MOTOR_PWM_STOP + delta, MOTOR_PWM_STOP + delta);
			}
			else
			{
				// CCW
				Motors_SetPWM(&motors, MOTOR_PWM_STOP - delta, MOTOR_PWM_STOP - delta);
			}
		}
	} break;

	case GOTO_DRIVE:
	{
		// Do not drive on an uninitialised or stale sensor frame. The ADC scan and
		// classification run independently and this state consumes only the cache.
		if (!obstacle_status_is_fresh(now_ms))
		{
			Motors_Stop(&motors);
			return;
		}

		if (obstacle_status.detected_mask != 0)
		{
			// Robot-classified rays are temporary blockages. Only static-classified
			// rays are committed to the persistent map. A side hit which is not on
			// the route is useful map information, but is not a collision condition.
			(void)mark_cached_static_obstacles();

			if (obstacle_mask_blocking_motion() != 0)
			{
				penalize_target_cell();
				Motors_SetPWM(&motors, MOTOR_PWM_MAX_BACKWARD, MOTOR_PWM_MAX_FORWARD);
				goto_backoff_until_ms = now_ms + OBSTACLE_BACKOFF_MS;
				goto_state = GOTO_BACKOFF;
				debug_due = 1;
				return;
			}

			// A newly mapped side obstacle may intersect a later waypoint. Replan
			// without backing away, because it is not an immediate collision.
			if (remaining_path_has_static_obstacle())
			{
				Motors_Stop(&motors);
				penalize_target_cell();
				path_len = 0;
				path_idx = 0;
				goto_state = GOTO_DONE;
				debug_due = 1;
				return;
			}
		}

		// If shared pose data shows that the next route cell is occupied, stop
		// before driving into it. Sensor-triggered backoff above takes priority
		// whenever the robots are already physically close.
		if (path_idx < path_len &&
				cell_is_occupied_by_peer(path_r[path_idx], path_c[path_idx], now_ms))
		{
			Motors_Stop(&motors);
			penalize_target_cell();
			goto_state = GOTO_DONE;
			return;
		}

		// while driving, if heading error grows too big -> stop and rotate again
		if (fabsf(e) > GOTO_THETA_DRIVE_MAX_RAD)
		{
			Motors_Stop(&motors);
			goto_rotate_started_ms = now_ms;
			goto_state = GOTO_ROTATE;
		}
		else
		{
			// keep driving forward
			Motors_SetPWM(&motors, MOTOR_PWM_MAX_FORWARD, MOTOR_PWM_MAX_BACKWARD);
		}
	} break;

	case GOTO_DONE:
	case GOTO_INACTIVE:
	default:
		// do nothing
		break;
	}
}

// not blocking command handling
// if command var != 'X':
//     if new_cmd flag is set:
//         command initialization
//         reset new_cmd flag
//     do 1 step of the command
//     if termination condition is met:
//         set command var to 'X'
// supported commands: H (reset experiment state), S (stop), Q (start coverage)
static void experiment_reset(void)
{
	Motors_Stop(&motors);

	memset(path_r, 0, sizeof(path_r));
	memset(path_c, 0, sizeof(path_c));
	path_len = 0;
	path_idx = 0;
	memset(params, 0, sizeof(params));
	cmd_end = 0;
	xt = 0.0f;
	yt = 0.0f;
	xt_i = 0;
	yt_i = 0;
	robot_theta_error = 0.0f;
	goto_rotate_started_ms = 0;
	goto_backoff_until_ms = 0;
	inactive_recheck_ms = 0;
	goto_state = GOTO_IDLE;

	memset(visits_map, 0, sizeof(visits_map));
	memset(penalties_map, 0, sizeof(penalties_map));
	memset(obstacles_map, 0, sizeof(obstacles_map));
	last_visit_r = -1;
	last_visit_c = -1;
	sce_rng_state = 0x9E3779B9u ^ ((uint32_t)MID * 0x85EBCA6Bu);

	// Clear the interpreted obstacle state, but deliberately keep the ADC scan
	// running and leave sensor thresholds/calibration untouched.
	memset(&obstacle_status, 0, sizeof(obstacle_status));
	for (uint32_t i = 0; i < IR_SENSOR_COUNT; i++)
	{
		obstacle_status.cell_r[i] = -1;
		obstacle_status.cell_c[i] = -1;
		obstacle_status.hit_x[i] = INVALID_POS;
		obstacle_status.hit_y[i] = INVALID_POS;
	}
	memset(obstacle_hit_streak, 0, sizeof(obstacle_hit_streak));
	memset(obstacle_clear_streak, 0, sizeof(obstacle_clear_streak));
	memset(obstacle_map_candidate_r, -1, sizeof(obstacle_map_candidate_r));
	memset(obstacle_map_candidate_c, -1, sizeof(obstacle_map_candidate_c));
	memset(obstacle_map_candidate_since_ms, 0,
			sizeof(obstacle_map_candidate_since_ms));
	obstacle_map_ready_mask = 0;

	memset(other_robots, 0, sizeof(other_robots));
	memset(other_robots_ids, 0, sizeof(other_robots_ids));
	memset(other_robots_last_seen_ms, 0, sizeof(other_robots_last_seen_ms));
	n_other_robots = 0;
	accept_peer_maps = 0;

	// Drop packets from the preceding run atomically. Fresh peer poses may still
	// be collected while idle, but M1 maps are ignored until Q starts the run.
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	opos_q_head = 0;
	opos_q_tail = 0;
	opos_q_count = 0;
	opos_drop_count = 0;
	posReady = 0;
	posBuffer[0] = '\0';
	__set_PRIMASK(primask);

	// Preserve the current pose until the requested camera update arrives, then
	// accept that first measurement as an absolute post-reset pose.
	camera_state = CAMERA_IDLE;
	camera_saved_pwm_l = MOTOR_PWM_STOP;
	camera_saved_pwm_r = MOTOR_PWM_STOP;
	camera_saved_command = 'X';
	camera_saved_goto_state = GOTO_IDLE;
	initial_pos = 1;
	cam_due = 1;
	broadcastPOS_due = 1;
	broadcastMAP_due = 1;
	debug_due = 1;

	command = 'X';
	new_cmd = 0;
}

void handle_command(void)
{
	if (command != 'X')
	{
		if (command == 'H')
		{
			experiment_reset();
		}
		else if (command == 'S')
		{
			// no initialization
			// stops the motors in 1 step and done
			Motors_Stop(&motors);
			path_len = 0;
			path_idx = 0;
			// S is a deliberate manual deactivation. Keep reporting INACTIVE so
			// the supervisor does not wait for this selected robot forever.
			goto_state = GOTO_INACTIVE;
			debug_due = 1;
			command = 'X';
			new_cmd = 0;
		}
		else if (command == 'Q')
		{
			if (new_cmd)
			{
				accept_peer_maps = 1;
				obstacles_map[0][0] = 1.0f; // known table-marker obstacle; it is not a visit
				last_visit_r = -1;
				last_visit_c = -1;
				broadcastPOS_due = 1;
				broadcastMAP_due = 1;
				goto_state = GOTO_DONE;   // wait to first select a cell in goto_done, then start the algorithm from there
				new_cmd = 0;
			}
			// copy robot_x and robot_y to local x,y
			uint32_t primask = __get_PRIMASK();
			float x,y,th;
			__disable_irq();
			x = robot_x;
			y = robot_y;
			th = robot_theta;
			__set_PRIMASK(primask);
			// mark the current place as visited; peer maps arrive in OPOS messages
			visits_map_update(x, y);

			uint32_t now_ms = HAL_GetTick();
			if (goto_state == GOTO_DONE ||
					(goto_state == GOTO_INACTIVE &&
					 (int32_t)(now_ms - inactive_recheck_ms) >= 0))
			{
				int active_peers = active_peer_count(now_ms);
				int sc = round_nearest((x - X0) / CELL);
				int sr = round_nearest((y - Y0) / CELL);
				uint8_t topology_reachable[ROWS][COLS];
				uint8_t route_reachable[ROWS][COLS];
				if (sc < 0) sc = 0;
				if (sc >= COLS) sc = COLS - 1;
				if (sr < 0) sr = 0;
				if (sr >= ROWS) sr = ROWS - 1;
				// Static topology decides whether this robot has any work at all.
				// Fresh peer cells are blocked only for the route used right now, so
				// moving robots cause a detour without becoming permanent obstacles.
				(void)compute_reachable_cells(sr, sc, topology_reachable, 0u, now_ms);
				(void)compute_reachable_cells(sr, sc, route_reachable, 1u, now_ms);

				// Only peers in this connected component compete for its remaining
				// cells. Peers behind a static obstacle must not make this robot yield.
				int competing_robots = 1;
				for (int i = 0; i < n_other_robots; i++)
				{
					int peer_r, peer_c;
					if (peer_is_fresh(i, now_ms) &&
							world_to_cell(other_robots[i][0], other_robots[i][1],
									&peer_r, &peer_c) &&
							topology_reachable[peer_r][peer_c])
						competing_robots++;
				}

				// find the next best cell to go
				float fittest = -1; // fitness value to be maximised, initially -1
				uint8_t have_candidate = 0;
				uint32_t tie_count = 0;
				float Ar, R, D, Dbar, fitness, dist, heading_error, cx, cy;
				uint8_t reachable_unvisited = 0;
	            // evaluates fitness for every cell on the grid
				for (int r = 0; r < ROWS; r++)
				{
					for (int c = 0; c < COLS; c++)
					{
						// skip obstacle cells entirely
						if (obstacles_map[r][c] >= 1.0f) {
						    continue;
						}
						// Coverage targets must be both unvisited and connected to the
						// robot through the current static-obstacle topology.
						if (visits_map[r][c] >= 1.0f || !topology_reachable[r][c])
							continue;
						reachable_unvisited++;

						// Select only cells reachable without crossing a fresh peer pose.
						if (!route_reachable[r][c])
							continue;

						// Never select a cell that a peer currently occupies. This is
						// live collision avoidance, not a reservation of future cells.
						if (cell_is_occupied_by_peer(r, c, now_ms))
							continue;

						cell_center(r, c, &cx, &cy);
						dist = dist_to_target(x, y, cx, cy); // direct distance
						if (dist < (CELL * 0.5f))
							continue; // avoid singularity and too close cells

						// Eq. (1) and Eq. (4): A^kappa = 1/(V + P + 1)^kappa.
						Ar = fpow_simple(visits_map[r][c] + penalties_map[r][c] + 1.0f,
								SCE_KAPPA);
						D = fpow_simple(dist, SCE_MU);
						Dbar = 0.0f;
						for (int i = 0; i < n_other_robots; i++)
						{
							if (!peer_is_fresh(i, now_ms)) continue;
							float dist_to_other_robot = dist_to_target(other_robots[i][0], other_robots[i][1], cx, cy);
							Dbar += dist_to_other_robot;
						}
						if (active_peers == 0) Dbar = 1.0f; // neutral dispersion term for one robot
						Dbar = fpow_simple(Dbar, SCE_LAMBDA);

						// Preserve the physical robot's straight-line preference while SCE
						// remains the coverage score. This avoids unnecessary stop-turn cycles.
						heading_error = heading_error_to_target(x, y, th, cx, cy);
						R = 1.0f + SCE_HEADING_WEIGHT * fabsf(heading_error) / (float)M_PI;
						fitness = Dbar / (Ar * D * R);

						float tie_epsilon = SCE_FITNESS_TIE_EPS * (1.0f + fabsf(fittest));
						int choose = 0;
						if (!have_candidate || fitness > fittest + tie_epsilon)
						{
							choose = 1;
							tie_count = 1;
						}
						else if (fittest > 0.0f && fabsf(fitness - fittest) <= tie_epsilon)
						{
							tie_count++;
							choose = ((sce_random() % tie_count) == 0u);
						}

						if (choose)
						{
							have_candidate = 1;
							fittest = fitness; // candidate this cell as the best cell so far (inc. its properties)
							// index of the best cell for penalties (global)
							xt_i = c;
							yt_i = r;
						}
					}
				}

				// When fewer reachable cells remain than competing robots, only the
				// closest robot should pursue each selected final cell. A deterministic
				// MID tie-break prevents equal-distance robots from both yielding.
				uint8_t yield_final_cells = 0;
				if (reachable_unvisited < competing_robots)
				{
					if (!have_candidate)
					{
						yield_final_cells = 1;
					}
					else
					{
						cell_center(yt_i, xt_i, &cx, &cy);
						float own_dist = dist_to_target(x, y, cx, cy);
						for (int i = 0; i < n_other_robots; i++)
						{
							int peer_r, peer_c;
							if (!peer_is_fresh(i, now_ms) ||
									!world_to_cell(other_robots[i][0], other_robots[i][1],
											&peer_r, &peer_c) ||
									!topology_reachable[peer_r][peer_c])
								continue;

							float peer_dist = dist_to_target(other_robots[i][0],
									other_robots[i][1], cx, cy);
							if (peer_dist + FINAL_CELL_DISTANCE_TIE_M < own_dist ||
									(fabsf(peer_dist - own_dist) <=
											FINAL_CELL_DISTANCE_TIE_M &&
									 other_robots_ids[i] < MID))
							{
								yield_final_cells = 1;
								break;
							}
						}
					}
				}

				if (have_candidate && !yield_final_cells)
				{
					// plan
					if (!astar_plan_cells(sr, sc, yt_i, xt_i)) {
					    // A peer may have moved after the reachability snapshot. Retry
					    // without penalising an otherwise valid coverage target.
					    path_len = 0;
					    path_idx = 0;
					    goto_state = GOTO_DONE;
					} else {
					    cell_center(path_r[0], path_c[0], &xt, &yt);
					    Motors_Stop(&motors);
					    goto_rotate_started_ms = now_ms;
					    goto_state = GOTO_ROTATE;
					    debug_due = 1;
					}
				}
				else if (reachable_unvisited == 0 || yield_final_cells)
				{
					// Either this component is complete or a closer peer owns the scarce
					// final work. Keep Q alive so map/pose updates can reactivate us.
					Motors_Stop(&motors);
					path_len = 0;
					path_idx = 0;
					if (goto_state != GOTO_INACTIVE)
						debug_due = 1;
					goto_state = GOTO_INACTIVE;
					inactive_recheck_ms = now_ms + INACTIVE_RECHECK_MS;
				}
				else
				{
					// Reachable work exists but every candidate is temporarily occupied
					// by a peer. Stay ready and retry without claiming inactivity.
					goto_state = GOTO_DONE;
				}
			}

			// run one step of gotoXY algorithm
			gotoXY();
		}
	}
}

static int opos_queue_pop(char *dst, size_t dst_size)
{
	int have_message = 0;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if (opos_q_count > 0)
	{
		uint8_t slot = opos_q_tail;
		strncpy(dst, opos_queue[slot], dst_size);
		dst[dst_size - 1] = '\0';
		opos_q_tail = (uint8_t)((slot + 1u) % OPOS_QUEUE_DEPTH);
		opos_q_count--;
		have_message = 1;
	}

	__set_PRIMASK(primask);
	return have_message;
}

static int parse_row_mask(const char *token, uint16_t *mask)
{
	char *end = NULL;
	unsigned long value = strtoul(token, &end, 16);
	unsigned long valid_bits = (1UL << COLS) - 1UL;

	if (end == token || *end != '\0' || value > valid_bits)
		return 0;

	*mask = (uint16_t)value;
	return 1;
}

static void merge_full_maps(const uint16_t visit_masks[ROWS],
		const uint16_t obstacle_masks[ROWS])
{
	for (int r = 0; r < ROWS; r++)
	{
		for (int c = 0; c < COLS; c++)
		{
			uint16_t bit = (uint16_t)(1u << c);
			if (visit_masks[r] & bit)
				visits_map[r][c] = MAX_VISIT_AND_PENALTY_COUNT;

			// A visit, whether local or received, proves that this cell is
			// traversable and therefore overrides every obstacle report.
			if (visits_map[r][c] >= 1.0f)
				obstacles_map[r][c] = 0.0f;
			else if (obstacle_masks[r] & bit)
				obstacles_map[r][c] = 1.0f;
		}
	}
}

static void handle_opos_line(char *local)
{
	char *save = NULL;
	char *tok = strtok_r(local, "#", &save);
	if (!tok || strcmp(tok, "OPOS") != 0) return;

	tok = strtok_r(NULL, "#", &save); if (!tok) return;
	char *id_end = NULL;
	long parsed_id = strtol(tok, &id_end, 10);
	if (id_end == tok || *id_end != '\0' || parsed_id < 0 || parsed_id > 255 ||
			parsed_id == MID || parsed_id == INVALID_MID)
		return;
	uint8_t id = (uint8_t)parsed_id;

	tok = strtok_r(NULL, "#", &save); if (!tok) return;
	float ox = (float)atof(tok);

	tok = strtok_r(NULL, "#", &save); if (!tok) return;
	float oy = (float)atof(tok);

	int peer_index = -1;
	for (int i = 0; i < n_other_robots; i++)
	{
		if (other_robots_ids[i] == id)
		{
			peer_index = i;
			break;
		}
	}
	if (peer_index < 0 && n_other_robots < MAX_OTHER_ROBOTS)
	{
		peer_index = n_other_robots;
		other_robots_ids[peer_index] = id;
		n_other_robots++;
	}
	if (peer_index >= 0)
	{
		other_robots[peer_index][0] = ox;
		other_robots[peer_index][1] = oy;
		other_robots_last_seen_ms[peer_index] = HAL_GetTick();
	}

	// M1 carries one visited bitmap and one obstacle bitmap for every grid row.
	tok = strtok_r(NULL, "#", &save);
	if (tok && strcmp(tok, "P1") == 0)
		return;

	if (tok && strcmp(tok, "M1") == 0)
	{
		// H resets experiment memory and closes this gate. Continue accepting
		// pose-only information while idle, but never merge a previous run's map.
		if (!accept_peer_maps)
			return;

		uint16_t visit_masks[ROWS];
		uint16_t obstacle_masks[ROWS];

		tok = strtok_r(NULL, "#", &save);
		if (!tok || strcmp(tok, "V") != 0) return;
		for (int r = 0; r < ROWS; r++)
		{
			tok = strtok_r(NULL, "#", &save);
			if (!tok || !parse_row_mask(tok, &visit_masks[r])) return;
		}

		tok = strtok_r(NULL, "#", &save);
		if (!tok || strcmp(tok, "O") != 0) return;
		for (int r = 0; r < ROWS; r++)
		{
			tok = strtok_r(NULL, "#", &save);
			if (!tok || !parse_row_mask(tok, &obstacle_masks[r])) return;
		}

		int route_active = (command == 'Q' &&
				(goto_state == GOTO_ROTATE || goto_state == GOTO_DRIVE));
		int target_was_visited = 1;
		int next_was_blocked = 1;
		int next_r = -1;
		int next_c = -1;

		if (route_active)
		{
			target_was_visited = (visits_map[yt_i][xt_i] >= 1.0f);
			if (path_idx < path_len)
			{
				next_r = path_r[path_idx];
				next_c = path_c[path_idx];
				next_was_blocked = (obstacles_map[next_r][next_c] >= 1.0f);
			}
		}

		merge_full_maps(visit_masks, obstacle_masks);

		// Do not finish a route whose target another robot has just covered.
		// Likewise, abandon a route when a newly shared obstacle blocks its
		// next step. The next main-loop pass performs a fresh SCE selection.
		if (route_active &&
				((!target_was_visited && visits_map[yt_i][xt_i] >= 1.0f) ||
				 (!next_was_blocked && next_r >= 0 &&
						obstacles_map[next_r][next_c] >= 1.0f)))
		{
			Motors_Stop(&motors);
			path_len = 0;
			path_idx = 0;
			goto_state = GOTO_DONE;
		}
		return;
	}

	// Backward compatibility with legacy OPOS#id#x#y#[r#c]... packets.
	while (tok)
	{
		char *tr = tok;
		char *tc = strtok_r(NULL, "#", &save);
		if (!tc) break;

		char *r_end = NULL;
		char *c_end = NULL;
		long r = strtol(tr, &r_end, 10);
		long c = strtol(tc, &c_end, 10);
		if (r_end != tr && *r_end == '\0' && c_end != tc && *c_end == '\0' &&
				r >= 0 && r < ROWS && c >= 0 && c < COLS)
			visits_map[r][c] = MAX_VISIT_AND_PENALTY_COUNT;

		tok = strtok_r(NULL, "#", &save);
	}
}

// Drain all queued peer messages so a complete multi-robot burst is processed.
void handle_opos_if_ready(void)
{
	char local[OPOS_MESSAGE_SIZE];
	while (opos_queue_pop(local, sizeof(local)))
		handle_opos_line(local);
}

// Sends a fast pose-only packet or a slower full SCE state packet.
void broadcast_pos(uint8_t include_maps)
{
	char tx[256];
	int len = 0;
	uint16_t visit_masks[ROWS] = {0};
	uint16_t obstacle_masks[ROWS] = {0};

	float x, y;
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		x = robot_x;
		y = robot_y;
		__set_PRIMASK(primask);
	}

	if (!include_maps)
	{
		len = snprintf(tx, sizeof(tx), "BPOS#%d#%.3f#%.3f#P1\n", MID, x, y);
		if (len > 0 && len < (int)sizeof(tx))
			HAL_UART_Transmit(&huart1, (uint8_t*)tx, (uint16_t)len, 50);
		return;
	}

	for (int r = 0; r < ROWS; r++)
	{
		for (int c = 0; c < COLS; c++)
		{
			uint16_t bit = (uint16_t)(1u << c);
			if (visits_map[r][c] >= 1.0f)
				visit_masks[r] |= bit;
			else if (obstacles_map[r][c] >= 1.0f)
				obstacle_masks[r] |= bit;
		}
	}

	len = snprintf(tx, sizeof(tx), "BPOS#%d#%.3f#%.3f#M1#V", MID, x, y);
	if (len < 0 || len >= (int)sizeof(tx)) return;

	for (int r = 0; r < ROWS; r++)
	{
		int n = snprintf(tx + len, sizeof(tx) - (size_t)len, "#%03X",
				(unsigned int)visit_masks[r]);
		if (n < 0 || n >= (int)(sizeof(tx) - (size_t)len)) return;
		len += n;
	}

	{
		int n = snprintf(tx + len, sizeof(tx) - (size_t)len, "#O");
		if (n < 0 || n >= (int)(sizeof(tx) - (size_t)len)) return;
		len += n;
	}

	for (int r = 0; r < ROWS; r++)
	{
		int n = snprintf(tx + len, sizeof(tx) - (size_t)len, "#%03X",
				(unsigned int)obstacle_masks[r]);
		if (n < 0 || n >= (int)(sizeof(tx) - (size_t)len)) return;
		len += n;
	}

	if (len < (int)sizeof(tx) - 2) {
		tx[len++] = '\n';
		tx[len] = '\0';
	} else {
		return;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)tx, (uint16_t)len, 50);
}
// -------------------------------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	// PWM Timers for motors
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PWM motor right
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // PWM motor left
	Motors_Init(&motors, &htim3, TIM_CHANNEL_1, &htim4, TIM_CHANNEL_1); // initializes motors and set their speed to STOP
	//  set_M_speeds(MOTOR_PWM_MAX_BACKWARD, MOTOR_PWM_MAX_BACKWARD); // CCW
	//  set_M_speeds(MOTOR_PWM_MAX_FORWARD, MOTOR_PWM_MAX_FORWARD); // CW
	//  set_M_speeds(MOTOR_PWM_MAX_FORWARD, MOTOR_PWM_MAX_BACKWARD); // F
	//  set_M_speeds(MOTOR_PWM_MAX_BACKWARD, MOTOR_PWM_MAX_FORWARD); // B

	// Init ADC and its DMAs for distance sensors
	ADC_Scan_Init();

	// set interrupt for USART1
	HAL_UART_Receive_IT(&huart1, &rxByte, 1);

	// set timer interrupt for odom and cam pos poll
	HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		uint32_t odom_n = 0, cam_n = 0, broadcastPOS_n = 0, broadcastMAP_n = 0;
		uint8_t debug_n = 0;
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		odom_n = odom_due;  odom_due = 0;
		cam_n = cam_due;   cam_due = 0;
		broadcastPOS_n = broadcastPOS_due; broadcastPOS_due = 0;
		broadcastMAP_n = broadcastMAP_due; broadcastMAP_due = 0;
		debug_n = debug_due; debug_due = 0;
		__set_PRIMASK(primask);

		// if requested, update odometry
		if (odom_n>0) {
			update_odometry(); // very quick, just reads encoder counts and does odom calculations
		}
		// receive and parse commands if cmdReady flag is set
		// uses cmdBuffer
		// sets command var and params array and new_cmd flag
		parse_command_if_ready();
		// Reset must not wait behind an in-flight camera correction. This also
		// guarantees that motors stop before any lower-priority serial/debug work.
		if (command == 'H')
			experiment_reset();

		// handle other robots pos if available
		handle_opos_if_ready();

		// Interpret every completed IR scan, regardless of navigation state. Peer
		// messages are handled first so robot/static classification uses fresh poses.
		obstacle_status_update();

		// A full map packet also contains the current pose, so avoid a duplicate packet.
		if (broadcastMAP_n)
		{
			broadcast_pos(1);
		}
		else if (broadcastPOS_n)
		{
			broadcast_pos(0);
		}

		// This advances without blocking, so peer position exchange and sensing keep
		// running throughout camera settling and response waits.
		camera_correction_step((cam_n > 0) ? 1u : 0u);

		// Large debug frames are deferred while the ESP is synchronously obtaining a
		// camera pose; compact BPOS/M1 traffic above continues during that interval.
		if (debug_n)
		{
			if (camera_correction_busy())
				debug_due = 1;
			else
				debug_send_state();
		}

		// Keep motors settled until the non-blocking camera correction finishes.
		if (!camera_correction_busy())
			handle_command();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 83;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
