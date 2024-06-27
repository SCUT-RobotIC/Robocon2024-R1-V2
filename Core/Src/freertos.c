/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_MODEL.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "calculate.h"
#include "math.h"
#include "delay.h"
#include "solve.h"
#include "new_logic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VEL 1
#define ANG 2

/**
 * @brief motor definition
 * motor_data[0] - motor_data[3]: chassis motors
 * motor_data[4]: ball shooter motor
 * motor_data[5]: ball right motor
 * motor_data[6]: ball left motor
 * motor_data[7]: ball lift motor
 */

extern motor_measure_t *motor_data[8];
extern motor_measure_t *motor_data1[8];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern int receivefactor[4];
extern int factor[2];
extern int factor1[2];

extern DataPacket DataRe;
extern int16_t lx, ly, rx, ry, lp, rp;
extern uint8_t B1, B2;
extern uint8_t Cal_Parity;

extern uint8_t BUTTON_State;
extern uint8_t SWITCH_LF_State;
extern uint8_t SWITCH_LB_State;
extern uint8_t SWITCH_RF_State;
extern uint8_t SWITCH_RB_State;

extern double output[16];

double Vx, Vy, omega = 0;

/* Manual Control Temp Variables */
double temp_Vy, temp_omega = 0;
double temp_Vx = 0;
uint8_t temp_BUTTON_State = 0;

double Controller_Deadband = 500;

/* R1 Ball Definition */
double SHOOT_UP_TGT = 0;
double SHOOT_DOWN_TGT = 0;
double SHOOT_R_TGT = 0;
double SHOOT_L_TGT = 0;
double LIFT_TGT = 0;

int16_t shoot_up_out = 0;
int16_t shoot_r_out = 0;
int16_t shoot_l_out = 0;
int16_t lift_out = 0;
TGT_COOR TC;
REAL_COOR RC;
const int M3508_MAX = 8911;
const int M2006_MAX = 14976;

const int ROLL_init = 120, GIVE_init = 26;
int ROLL_ANG = ROLL_init, GIVE_ANG = GIVE_init;
int ROLL_state = 0, GIVE_state = 0;

/* R1 Clamp Definition */
float YAW_TGT[8] = {0};
float M_3508_YAW_TGT = 0;
float SPD_TGT[8] = {0}; //{-500,500}
float ANG_TGT[8] = {0};
float current_ang = 0;
int can_output[8] = {0};
uint8_t FLAG = 0;

float current_target[8] = {0};
float target_error[8] = {0};
float test_target = 0;

uint16_t HT_moto_yaw = 0;

uint8_t LOGIC_FLAG = 0;
uint8_t logic_state = 0;
uint8_t I2C_TRANS_FLAG = 0;
uint8_t M_3508_TRANS_FLAG = 0;
uint8_t HIGH_TROQUE_TRANS_FLAG = 0;

uint8_t GPIO_CHANGE_STATE_1;
uint8_t GPIO_CHANGE_STATE_2;
uint8_t GPIO_CHANGE_FLAG = 0;
MotorExtentTypeDef motorExtent = {
    .state = 0xab,
};

/* Upper Feedback UART */
int buff_len;
char TransmitBuffer[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ballTask */
osThreadId_t ballTaskHandle;
const osThreadAttr_t ballTask_attributes = {
  .name = "ballTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for clampTask */
osThreadId_t clampTaskHandle;
const osThreadAttr_t clampTask_attributes = {
  .name = "clampTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for upperFeedbackTa */
osThreadId_t upperFeedbackTaHandle;
const osThreadAttr_t upperFeedbackTa_attributes = {
  .name = "upperFeedbackTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime)
{
  uint16_t compare_value = 0;
  if (angle <= 180)
  {
    compare_value = 0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90;
    __HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
  }
}

void BALL_On(void)
{
  ROLL_ANG = 0;
  GIVE_ANG = 26;
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);
  SHOOT_UP_TGT = 0;
  SHOOT_DOWN_TGT = 8000;
  LIFT_TGT = -14000;
}

void BALL_Step(void)
{
  ROLL_ANG = 120;
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
  HAL_Delay(1000);

  GIVE_ANG = 80;
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);
  HAL_Delay(1000);

  BALL_On();
}


void BALL_Stop(void)
{
  SHOOT_UP_TGT = 0;
  SHOOT_DOWN_TGT = 0;
  SHOOT_R_TGT = 0;
  SHOOT_L_TGT = 0;
  LIFT_TGT = 0;
  ROLL_ANG = ROLL_init;
  GIVE_ANG = GIVE_init;

  output[CH1_5] = 0;
  output[CH1_6] = 0;
  output[CH1_7] = 0;
  CAN1_cmd_chassis(output[CH1_1], output[CH1_2], output[CH1_3], output[CH1_4], output[CH1_5], output[CH1_6], output[CH1_7], 0);
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_init, 20000, 20);
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_init, 20000, 20);
}

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void *argument);
void StartBallTask(void *argument);
void StartClampTask(void *argument);
void StartUpperFeedbackTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);

  /* creation of ballTask */
  ballTaskHandle = osThreadNew(StartBallTask, NULL, &ballTask_attributes);

  /* creation of clampTask */
  clampTaskHandle = osThreadNew(StartClampTask, NULL, &clampTask_attributes);

  /* creation of upperFeedbackTa */
  upperFeedbackTaHandle = osThreadNew(StartUpperFeedbackTask, NULL, &upperFeedbackTa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassisTask */
/**
 * @brief  Function implementing the chassisTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for (;;)
  {
    CAL_TXMESSAGE();
    if (SWITCH_LB_State == 1)
    {
      Vx = RC.Vx;
      Vy = RC.Vy;
      omega = RC.omega;
    }
    else if (SWITCH_LB_State == 0)
    {
      Vx = rx / 9;
      Vy = ry / 9;
      omega = lx / 9;

      if (Vx > Controller_Deadband)
      {
        Vx -= Controller_Deadband;
      }
      else if (Vx < -Controller_Deadband)
      {
        Vx += Controller_Deadband;
      }
      else
      {
        Vx = 0;
      }

      if (Vy > Controller_Deadband)
      {
        Vy -= Controller_Deadband;
      }
      else if (Vy < -Controller_Deadband)
      {
        Vy += Controller_Deadband;
      }
      else
      {
        Vy = 0;
      }

      if (omega > Controller_Deadband)
      {
        omega -= Controller_Deadband;
      }
      else if (omega < -Controller_Deadband)
      {
        omega += Controller_Deadband;
      }
      else
      {
        omega = 0;
      }
    }

    factor1[0]++;

    if (receivefactor[0] == 0) // æ²¡æ¥æ¶å°å°±å¢å æ å¿ä½
      factor[0]++;
    if (factor[0] > 300)
    {
      Vx = 0;
      Vy = 0;
      omega = 0;

      rx = 0;
      ry = 0;
      lx = 0;
      ly = 0;
      factor[0] = 301;
    } 
    if (receivefactor[0] == 1) 
      factor[0] = 0;

    if (factor1[0] == 50)
    {
      receivefactor[0] = 0; 
      factor1[0] = 0;
    }

    HAL_IWDG_Refresh(&hiwdg); 

    /* SPD TEST */

    get_msgn();

    set_mode(VEL, VEL, VEL, VEL, VEL, VEL, VEL,
             ANG, ANG, ANG, ANG, VEL, VEL, VEL);

    ctrlmotor(Vx, Vy, omega);
    osDelay(1);
  }

  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartBallTask */
/**
 * @brief Function implementing the BallTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBallTask */
void StartBallTask(void *argument)
{
  /* USER CODE BEGIN StartBallTask */
  /* Infinite loop */
  for (;;)
  {
    rtU.yaw_target_CH1_4 = LIFT_TGT;
    rtU.yaw_target_CH1_5 = -SHOOT_DOWN_TGT;
    rtU.yaw_target_CH1_6 = SHOOT_DOWN_TGT;


    switch (BUTTON_State)
    {

    case 1:
      // Emergency Stop
      BALL_Stop();
      break;

    case 2:
      // R1 Ball ON
      BALL_On();
      break;

    case 3:
      BALL_Step();
      break;
    default:
      break;
    }
    osDelay(1);
  }
  /* USER CODE END StartBallTask */
}

/* USER CODE BEGIN Header_StartClampTask */
/**
 * @brief Function implementing the clampTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartClampTask */
void StartClampTask(void *argument)
{
  /* USER CODE BEGIN StartClampTask */
  /* Infinite loop */
  for (;;)
  {
    if (RC.action == CLAMP_PINCH)
    {
    }

    LOGIC();

    vTaskDelay(100);

    osDelay(1);
  }
  /* USER CODE END StartClampTask */
}

/* USER CODE BEGIN Header_StartUpperFeedbackTask */
/**
 * @brief Function implementing the upperFeedbackTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUpperFeedbackTask */
void StartUpperFeedbackTask(void *argument)
{
  /* USER CODE BEGIN StartUpperFeedbackTask */
  /* Infinite loop */
  for (;;)
  {
    buff_len = sprintf(TransmitBuffer, "fg %d %d %d %d %d %d %d %d\r\n", motor_data[0]->speed_rpm, motor_data[1]->speed_rpm, motor_data[2]->speed_rpm, motor_data[3]->speed_rpm, (int)rtU.yaw_target_CH1_1, (int)rtU.yaw_target_CH1_2, (int)rtU.yaw_target_CH1_3, (int)rtU.yaw_target_CH1_4);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)TransmitBuffer, buff_len);

    osDelay(1);
  }
  /* USER CODE END StartUpperFeedbackTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

