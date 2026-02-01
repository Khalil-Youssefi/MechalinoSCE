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
#include "robot_config.h"
#include "motor.h"
#include "gotoxy.h"
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
    GOTO_DONE
} goto_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Motors
#define MOTOR_PWM_STOP 1500
#define MOTOR_PWM_MAX_FORWARD 1300
#define MOTOR_PWM_MAX_BACKWARD 1700
// drive/rotate speeds
#define ROT_CW_L   (MOTOR_PWM_MAX_FORWARD)
#define ROT_CW_R   (MOTOR_PWM_MAX_FORWARD)
#define ROT_CCW_L  (MOTOR_PWM_MAX_BACKWARD)
#define ROT_CCW_R  (MOTOR_PWM_MAX_BACKWARD)
#define FWD_L      (MOTOR_PWM_MAX_FORWARD)
#define FWD_R      (MOTOR_PWM_MAX_BACKWARD)

// Encoder
#define ENCODER_RES 20

// Distance sensors
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
#define GOTO_THETA_OK_RAD        (3.0f * (float)M_PI / 180.0f)
#define GOTO_THETA_DRIVE_MAX_RAD (5.0f * (float)M_PI / 180.0f)
#define GOTO_DIST_OK_M           (0.02f)

// grid parameters
#define CELL   0.15f
#define X0     0.15f
#define Y0     0.15f
#define COLS   11
#define ROWS   4

// grid special values
#define INVALID_POS -1000.0f
#define MAX_VISIT_AND_PENALTY_COUNT 1000.0f

// inter-swarm communication
#define MID 16                                                    // Mechalino ID (MID)
#define MAX_OTHER_ROBOTS 4                                        // max number of others (Maximum Swarm Size=5)
#define INVALID_MID 222

// recovery window for broadcasts
#define REC_WINDOW 5

// obstacle avoidance
#define OBSTACLE_DIST_M       0.17f
#define OBSTACLE_MARK_R       0.075f

#define OBSTACLE_TH0_MV       900u                                // front
#define OBSTACLE_TH1_MV       1200u                               // front-right
#define OBSTACLE_TH2_MV       1200u                               // front-left

#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)

// Sensors direction relative to robot forward direction
#define S0_OFF_RAD  (0.0f)
#define S1_OFF_RAD  (DEG2RAD(+45.0f))                             // s1 is +45 deg
#define S2_OFF_RAD  (DEG2RAD(-45.0f))                             // s2 is -45 deg
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
volatile int16_t encoder_right_count = 0;
volatile int16_t encoder_left_count = 0;

// Distance sensors
volatile uint16_t adc_buffer[IRD_NUM_SAMPLES];
volatile uint16_t adc_readings_off[3];  // active DMUX channels are 0, 1 and 7 when IR LED is off
volatile uint16_t adc_readings_on[3];  // active DMUX channels are 0, 1 and 7 when IR LED is on
volatile uint16_t adc_readings[3];  // active DMUX channels are 0, 1 and 7 [difference between on and off]
uint8_t current_step = 0; // step 0: IR LED off, step 1: IR LED on
uint8_t current_dmux_index = 0;
const uint8_t dmux_channels[3] = {0, 1, 7}; // active DMUX channels

// Serial communication with ESP8266
uint8_t rxByte;
char rxBuffer[256];
uint8_t rxIndex = 0;
volatile uint8_t posReady = 0;
volatile uint8_t oposReady = 0;
volatile uint8_t cmdReady  = 0;
char cmdBuffer[256];
char posBuffer[256];

// robot position
float robot_x = 0.0f;
float robot_y = 0.0f;
float robot_theta = 0.0f;

uint8_t initial_pos = 1;
volatile uint8_t broadcastPOS_due = 0; // flag to broadcast position
volatile uint8_t odom_due = 0; // flag to request odom update
volatile uint32_t cam_due  = 0; // flag for request cam pos update

// Remote control
char command = 'X';
uint8_t new_cmd = 0;
float params[5];
uint32_t cmd_end;

// GOTO XY

volatile goto_state_t goto_state = GOTO_IDLE;

float xt,yt; // target point (center of the target cell)
int xt_i, yt_i; // grid indices of the target cell

// SCE memory
float visits_map[ROWS][COLS] = {0};
float penalties_map[ROWS][COLS] = {0};

// inter swarm communication
// posBuffer is used also as oposBuffer
float other_robots[MAX_OTHER_ROBOTS][2];
uint8_t other_robots_ids[MAX_OTHER_ROBOTS] = {0};
uint8_t n_other_robots = 0;

// recovery window for broadcasts
static int16_t rec_cells[REC_WINDOW][2];  // [i][0]=r, [i][1]=c
static uint8_t rec_head = 0;
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
void request_camera_correction(void);

/* --- DMUX + ADC scanning --- */
void Set_DMUX_Address(uint8_t address);
void ADC_Scan_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/* --- UART RX + command parsing --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void parse_command_if_ready(void);

/* --- SCE memory / recovery window --- */
static void   rec_init(void);
static void   rec_push_cell(int r, int c);
static inline int round_nearest(float v);

void  penalize_target_cell(void);
void  discount_penalties(void);
void  visits_map_update(float x, float y);
float fpow_simple(float base, unsigned exp);

/* --- Obstacle projection + visited marking --- */
static inline void unit_vec_from_theta(float th, float off, float *ux, float *uy);

static inline void obstacle_pos_from_pos_and_offset(float x, float y, float th,
                                                    float dist_m, float off_rad,
                                                    float *ox, float *oy);

static void visits_map_mark_radius(float ox, float oy, float r);
static void mark_obstacle_cells_from_three_sensors(uint16_t s0, uint16_t s1, uint16_t s2);

/* --- GOTO XY helpers + state machine --- */
static float wrap_pi(float a);

static float desired_theta_to_target(float x, float y, float th, float tx, float ty);
static float heading_error_to_target(float x, float y, float th, float tx, float ty);
static float dist_to_target(float x, float y, float tx, float ty);

static inline int obstacle_in_front(void);

void gotoXY(void);
void handle_command(void);

/* --- Inter-swarm / broadcasts --- */
void handle_opos_if_ready(void);
void broadcast_pos(void);
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

      static uint16_t div300 = 0;
      if (++div300 >= 300) {     // 3 seconds @ 100 Hz
        div300 = 0;
        broadcastPOS_due = 1;
      }
    }
}

// Odometry calculations
void update_odometry(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	int16_t curr_left  = encoder_left_count;
	int16_t curr_right = encoder_right_count;
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
}

// request camera correction (Serial to ESP)
void request_camera_correction(void)
{
	// if command P as active, then don't poll cam pos, because of Serial conflicts
	if (command == 'P')
		return;

    // 1) motor_L and motor_R keep current commanded motor PWM "speeds"
	uint16_t motor_L = motors.pwm_l;
	uint16_t motor_R = motors.pwm_r;

    // 2) stop robot
	Motors_Stop(&motors);

    // 3) let mechanics settle
    HAL_Delay(STOP_SETTLE_MS);

    // 4) request pos from ESP (serial)
    const char req[] = "POS?\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)req, (uint16_t)(sizeof(req) - 1), 50);

    // 5) wait for reply (max POS_WAIT_MS); if no reply -> ignore
    // Clear any previous message atomically
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        posReady = 0;
        posBuffer[0] = '\0';
        __set_PRIMASK(primask);
    }

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < POS_WAIT_MS)
    {
        if (posReady)
        {
            char local[256];

            // copy buffer atomically and clear flag
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            posReady = 0;
            strncpy(local, posBuffer, sizeof(local));
            local[sizeof(local) - 1] = '\0';
            __set_PRIMASK(primask);

            float x_meas, y_meas, th_meas;
            if (parse_pos_reply(local, &x_meas, &y_meas, &th_meas))
            {
                // 6) update pos with LPF
                lpf_update_pos(x_meas, y_meas, th_meas);
            }
            break; // either parsed or ignored; in both cases stop waiting
        }
    }

    // 7) resume motion (restore previous PWM commands)
    Motors_SetPWM(&motors, motor_L, motor_R);
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
        uint32_t sum = 0;
        for(int i = 0; i < IRD_NUM_SAMPLES; i++) {
            sum += adc_buffer[i];
        }
        float miliVolts = (sum * 3300.0f) / (IRD_NUM_SAMPLES * 4095.0f);

        // Store the reading based on current step
        if(current_step == 0)  // Step 0: DMUX disabled (OFF reading)
        {
            adc_readings_off[current_dmux_index] = (uint16_t)miliVolts;
            current_step = 1;

            // Enable DMUX for ON reading
            HAL_GPIO_WritePin(DMUX_PORT, DMUX_EN, GPIO_PIN_SET);
        }
        else  // Step 1: DMUX enabled (ON reading)
        {
            adc_readings_on[current_dmux_index] = (uint16_t)miliVolts;

            // Calculate difference
            if (adc_readings_off[current_dmux_index] < adc_readings_on[current_dmux_index])
            	adc_readings[current_dmux_index] = adc_readings_on[current_dmux_index] - adc_readings_off[current_dmux_index];
            else
            	adc_readings[current_dmux_index] = 0; // invalid reading

            current_step = 0;

            // Disable DMUX
            HAL_GPIO_WritePin(DMUX_PORT, DMUX_EN, GPIO_PIN_RESET);

            // Move to next channel
            current_dmux_index++;
            if(current_dmux_index >= 3) {
                current_dmux_index = 0;
                // All 3 channels complete!
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
		if (rxByte == '\n')
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
                if (!oposReady)
                {
                    strncpy(posBuffer, rxBuffer, sizeof(posBuffer));
                    posBuffer[sizeof(posBuffer) - 1] = '\0';
                    oposReady = 1;
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

static void rec_init(void)
{
    for (int i = 0; i < REC_WINDOW; i++) {
        rec_cells[i][0] = -1;
        rec_cells[i][1] = -1;
    }
    rec_head = 0;
}

static void rec_push_cell(int r, int c)
{
    rec_cells[rec_head][0] = (int16_t)r;
    rec_cells[rec_head][1] = (int16_t)c;

    rec_head++;
    if (rec_head >= REC_WINDOW) rec_head = 0;
}

static inline int round_nearest(float v)
{
    return (v >= 0.0f) ? (int)(v + 0.5f) : (int)(v - 0.5f);
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
	        penalties_map[i][j] *= 0.5f;
	    }
	}
}

void visits_map_update(float x, float y)
{
    int c = round_nearest((x - X0) / CELL);
    int r = round_nearest((y - Y0) / CELL);

    /* clamp to valid grid */
    if (c < 0) c = 0;
    if (c >= COLS) c = COLS - 1;

    if (r < 0) r = 0;
    if (r >= ROWS) r = ROWS - 1;

    if (visits_map[r][c] == 0)
    {
    	discount_penalties();
    	// update recovey window
    	rec_push_cell(r, c);
    }
    if (visits_map[r][c] < MAX_VISIT_AND_PENALTY_COUNT)
    	visits_map[r][c] += 1.0f;
}

float fpow_simple(float base, unsigned exp)
{
    float r = 1.0f;
    while (exp--) r *= base;
    return r;
}

static inline void unit_vec_from_theta(float th, float off, float *ux, float *uy)
{
    // Your robot forward is (theta + pi/2). Add sensor offset around that.
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

static void visits_map_mark_radius(float ox, float oy, float r)
{
    float r2 = r * r;

    for (int rr = 0; rr < ROWS; rr++)
    {
        for (int cc = 0; cc < COLS; cc++)
        {
            float cx = cc * CELL + X0;
            float cy = rr * CELL + Y0;

            float dx = cx - ox;
            float dy = cy - oy;

            if ((dx*dx + dy*dy) <= r2)
            {
                if (visits_map[rr][cc] < 1.0f)
                    visits_map[rr][cc] = 1.0f;
            }
        }
    }
}

static void mark_obstacle_cells_from_three_sensors(uint16_t s0, uint16_t s1, uint16_t s2)
{
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

    // For each sensor above its threshold, project and mark
    float ox, oy;

    if (s0 > OBSTACLE_TH0_MV)
    {
        obstacle_pos_from_pos_and_offset(x, y, th, OBSTACLE_DIST_M, S0_OFF_RAD, &ox, &oy);
        visits_map_mark_radius(ox, oy, OBSTACLE_MARK_R);
    }

    if (s1 > OBSTACLE_TH1_MV)
    {
        obstacle_pos_from_pos_and_offset(x, y, th, OBSTACLE_DIST_M, S1_OFF_RAD, &ox, &oy);
        visits_map_mark_radius(ox, oy, OBSTACLE_MARK_R);
    }

    if (s2 > OBSTACLE_TH2_MV)
    {
        obstacle_pos_from_pos_and_offset(x, y, th, OBSTACLE_DIST_M, S2_OFF_RAD, &ox, &oy);
        visits_map_mark_radius(ox, oy, OBSTACLE_MARK_R);
    }
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

static inline int obstacle_in_front(void)
{
    uint16_t s0, s1, s2;

    // atomic snapshot
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s0 = adc_readings[0];   // front
    s1 = adc_readings[1];   // front-right
    s2 = adc_readings[2];   // front-left
    __set_PRIMASK(primask);

    // no obstacle
    if (s0 <= OBSTACLE_TH0_MV && s1 <= OBSTACLE_TH1_MV && s2 <= OBSTACLE_TH2_MV) {
        return 0;
    }

    // any obstacle detected
    return 1;
}

// non-blocking step function
void gotoXY()
{
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

    // stop condition
    if (d <= GOTO_DIST_OK_M)
    {
    	Motors_Stop(&motors);
        goto_state = GOTO_DONE;
        return;
    }

    switch (goto_state)
    {
        case GOTO_ROTATE:
        {
            // if aligned enough -> start driving
            if (fabsf(e) <= GOTO_THETA_OK_RAD)
            {
            	Motors_SetPWM(&motors, MOTOR_PWM_MAX_FORWARD, MOTOR_PWM_MAX_BACKWARD);
                goto_state = GOTO_DRIVE;
            }
            else
            {
                // rotate towards target
                if (e > 0.0f)
                {
					// CW
					Motors_SetPWM(&motors, ROT_CW_L, ROT_CW_R);
                }
                else
                {
					// CCW
					Motors_SetPWM(&motors, ROT_CCW_L, ROT_CCW_R);
                }
            }
        } break;

        case GOTO_DRIVE:
        {
        	uint16_t s0, s1, s2;
			// atomic snapshot of sensors
			{
				uint32_t primask = __get_PRIMASK();
				__disable_irq();
				s0 = adc_readings[0];   // front
				s1 = adc_readings[1];   // +45°
				s2 = adc_readings[2];   // -45°
				__set_PRIMASK(primask);
			}

			int any_obstacle = (s0 > OBSTACLE_TH0_MV) || (s1 > OBSTACLE_TH1_MV) || (s2 > OBSTACLE_TH2_MV);

			if (any_obstacle)
			{
				// mark obstacle footprint(s) in the grid using all triggered sensors
				mark_obstacle_cells_from_three_sensors(s0, s1, s2);

				penalize_target_cell();
				Motors_Stop(&motors);
				goto_state = GOTO_DONE;
			}
            // while driving, if heading error grows too big -> stop and rotate again
        	else if (fabsf(e) > GOTO_THETA_DRIVE_MAX_RAD)
            {
				Motors_Stop(&motors);
                goto_state = GOTO_ROTATE;
            }
            else
            {
                // keep driving forward
            	Motors_SetPWM(&motors, MOTOR_PWM_MAX_FORWARD, MOTOR_PWM_MAX_BACKWARD);
            }
        } break;

        case GOTO_DONE:
        default:
            // do nothing
            break;
    }
}

void handle_command(void)
{
	if (command != 'X')
	{
	  if (command == 'S')
	  {
		  Motors_Stop(&motors);
		  command = 'X';
		  new_cmd = 0;
	  }
	  else if (command == 'Q')
	  {
	      if (new_cmd)
	      {
	    	  visits_map[0][0] = 100; // mark table marker place az visited
	    	  goto_state = GOTO_DONE;   // start by rotating to face target
	          new_cmd = 0;
	      }
		  // mark current place as visited for this robot and other known robots
		  uint32_t primask = __get_PRIMASK();
		  float x,y;
		  __disable_irq();
		  x = robot_x;
		  y = robot_y;
		  __set_PRIMASK(primask);
		  visits_map_update(x, y);
		  for (int i = 0; i < n_other_robots; i++)
		  {
			  visits_map_update(other_robots[i][0], other_robots[i][1]);
		  }

	      if (goto_state == GOTO_DONE)
	      {
			  // find the next best cell to go
			  float best_x, best_y;
			  int closer_bots_f = 0;
			  float fittest = -1;
			  float Ar, D, Dbar,fitness,dist,cx,cy;
			  uint8_t unvisited = 0;
			  for (int r = 0; r < 4; r++)
			  {
				  for (int c = 0; c < 11; c++)
				  {
					if (visits_map[r][c] < 1)
					  unvisited += 1;
					Ar = fpow_simple(visits_map[r][c] + penalties_map[r][c] + 1, 10);
					cx = c * 0.15f + 0.15f;
					cy = r * 0.15f + 0.15f;
					dist = dist_to_target(x, y, cx, cy);
					D = fpow_simple(dist, 2);
					Dbar = 1;
				    int closer_bots = 0;
					for (int i = 0; i < n_other_robots; i++)
					{
						float dist_to_other_robot = dist_to_target(other_robots[i][0], other_robots[i][1], cx, cy);
						Dbar += dist_to_other_robot;
						if (dist_to_other_robot < dist)
							closer_bots++;
					}
					Dbar = fpow_simple(Dbar, 2);
					fitness = Dbar / (Ar*D);
					if (fitness>fittest && dist>=0.075f) //TODO: 0.075 is half cell size
					{
						  fittest = fitness;
						  closer_bots_f = closer_bots;
						  best_x = cx;
						  best_y = cy;
						  xt_i = c;
						  yt_i = r;
					}
				  }
			  }

			  // if number of unvisited cells are less than the number of robots, robots should avoid moving if they are not the closest bot to the unvisited cells
			  if (unvisited < (n_other_robots+1) && closer_bots_f>0)
				  fittest = -1;
			  // fitness maybe -1 (no cell celected)
			  if (fittest > 0)
			  {
				xt = best_x;
				yt = best_y;
				Motors_Stop(&motors);
				goto_state = GOTO_ROTATE;
			  }

		      // when done, clear command
		      if (unvisited == 0)
		      {
		          goto_state = GOTO_IDLE;
		          command = 'X';
		      }
	      }

	      // run one step each loop
	      gotoXY();
	  }
	}
}


void handle_opos_if_ready(void)
{
    if (!oposReady) return;

    char local[256];

    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        oposReady = 0;
        strncpy(local, posBuffer, sizeof(local));
        local[sizeof(local) - 1] = '\0';
        __set_PRIMASK(primask);
    }

    // Expect: OPOS#id#x#y#[r#c]...
    // Tokenize by '#'
    char *save = NULL;
    char *tok = strtok_r(local, "#", &save);
    if (!tok || strcmp(tok, "OPOS") != 0) return;

    tok = strtok_r(NULL, "#", &save); if (!tok) return;
    int id = atoi(tok);

    tok = strtok_r(NULL, "#", &save); if (!tok) return;
    float ox = (float)atof(tok);

    tok = strtok_r(NULL, "#", &save); if (!tok) return;
    float oy = (float)atof(tok);

    // update other_robots list (same as your logic)
    int found = 0;
    for (int i = 0; i < n_other_robots; i++) {
        if (other_robots_ids[i] == id) {
            other_robots[i][0] = ox;
            other_robots[i][1] = oy;
            found = 1;
            break;
        }
    }
    if (!found && n_other_robots < MAX_OTHER_ROBOTS) {
        other_robots_ids[n_other_robots] = id;
        other_robots[n_other_robots][0] = ox;
        other_robots[n_other_robots][1] = oy;
        n_other_robots++;
    }

    // Parse remaining tokens as (r,c) pairs
    while (1) {
        char *tr = strtok_r(NULL, "#", &save);
        if (!tr) break;
        char *tc = strtok_r(NULL, "#", &save);
        if (!tc) break;

        int r = atoi(tr);
        int c = atoi(tc);
        if (r < 0 || r >= ROWS) continue;
        if (c < 0 || c >= COLS) continue;
        // do the update directlt here
        if (visits_map[r][c] < MAX_VISIT_AND_PENALTY_COUNT)
                visits_map[r][c] += 1.0f;
    }
}

void broadcast_pos(void)
{
    char tx[256];
    int len = 0;

    // atomic snapshot of pos (if you still want x,y for "current position" use)
    float x, y;
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        x = robot_x;
        y = robot_y;
        __set_PRIMASK(primask);
    }

    // atomic snapshot of recovery cells
    int16_t snap[REC_WINDOW][2];
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        for (int i = 0; i < REC_WINDOW; i++) {
            snap[i][0] = rec_cells[i][0];
            snap[i][1] = rec_cells[i][1];
        }
        __set_PRIMASK(primask);
    }

    // base
    len = snprintf(tx, sizeof(tx), "BPOS#%d#%.3f#%.3f", MID, x, y);
    if (len < 0 || len >= (int)sizeof(tx)) return;

    // order is not important
    for (int k = 0; k < REC_WINDOW; k++) {
        int r = snap[k][0];
        int c = snap[k][1];
        if (r < 0 || c < 0)
        	break; // skip the rest, this will happen only at the beginning that the window is not full

        int n = snprintf(tx + len, sizeof(tx) - (size_t)len, "#%d#%d", r, c);
        if (n < 0 || n >= (int)(sizeof(tx) - (size_t)len)) break;
        len += n;
    }

    // newline
    if (len < (int)sizeof(tx) - 2) {
        tx[len++] = '\n';
        tx[len] = '\0';
    } else {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)tx, (uint16_t)len, 50);
}


static inline int compute_avoid_rotation(int *rot_dir)
{
    // rot_dir: -1 = rotate right, +1 = rotate left, 0 = none

    uint16_t s0, s1, s2;

    // atomic snapshot
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s0 = adc_readings[0];   // front
    s1 = adc_readings[1];   // front-right
    s2 = adc_readings[2];   // front-left
    __set_PRIMASK(primask);

    if (s0 <= 600 && s1 <= 1000 && s2 <= 1000) {
        *rot_dir = 0;
        return 0;
    }

    if (s0 > 500) {
        *rot_dir = (s1 > s2) ? +1 : -1;
        return 1;
    }

    if (s1 > 500) {
        *rot_dir = +1;   // rotate left
        return 1;
    }

    if (s2 > 500) {
        *rot_dir = -1;   // rotate right
        return 1;
    }

    *rot_dir = 0;
    return 0;
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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  Motors_Init(&motors, &htim3, TIM_CHANNEL_1, &htim4, TIM_CHANNEL_1);
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

  rec_init(); // init recovery window for broadcasts

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t odom_n = 0, cam_n = 0, broadcastPOS_n = 0;
	  uint32_t primask = __get_PRIMASK();
	  __disable_irq();
	  odom_n = odom_due;  odom_due = 0;
	  cam_n  = cam_due;   cam_due  = 0;
	  broadcastPOS_n = broadcastPOS_due; broadcastPOS_due = 0;
	  __set_PRIMASK(primask);

	  // if requested, update odometry
	  if (odom_n>0) {
	      update_odometry();
	  }
	  // if requested, update from cam pos
	  if (cam_n)
	  {
	      request_camera_correction();
	  }
    // if due time, broadcast pos to wifi module
    if (broadcastPOS_n)
    {
        broadcast_pos();
    }

	  // receive and parse commands
	  parse_command_if_ready();

    // handle other robots pos if available
    handle_opos_if_ready();

	// handle commands
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
  htim5.Init.Period = 4;
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
