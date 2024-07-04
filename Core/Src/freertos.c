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
#include "sbus.h"

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
extern switch_enum SWITCH_LF_State;
extern switch_enum SWITCH_LB_State;
extern switch_enum SWITCH_RF_State;
extern switch_enum SWITCH_RB_State;
extern int MAXVAL;

extern switch_enum SWITCH_SBUS_LB_State;
extern switch_enum SWITCH_SBUS_RF_State;

/* Chassis Velocity Variables */
extern double output[16];
double Vx, Vy, omega = 0;
double V_Sum = 0;
double V_Sum_Last = 0;

/* Manual Control Temp Variables */

double Controller_Deadband = 300;

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

bool_T LOGIC_FLAG = off;
uint8_t logic_state = 0;
uint8_t I2C_TRANS_FLAG = 0;
uint8_t M_3508_TRANS_FLAG = 0;

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
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ballTask */
osThreadId_t ballTaskHandle;
const osThreadAttr_t ballTask_attributes = {
    .name = "ballTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for clampTask */
osThreadId_t clampTaskHandle;
const osThreadAttr_t clampTask_attributes = {
    .name = "clampTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for upperFeedbackTa */
osThreadId_t upperFeedbackTaHandle;
const osThreadAttr_t upperFeedbackTa_attributes = {
    .name = "upperFeedbackTa",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for SBUSTask */
osThreadId_t SBUSTaskHandle;
const osThreadAttr_t SBUSTask_attributes = {
    .name = "SBUSTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime)
{
  uint16_t compare_value = 0;
  if (angle <= 180)
  {
    compare_value = 20000 - (0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90); // 20000-(500+angle*11.11�????)
    __HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
  }
}

void BALL_On(void)
{
  ROLL_ANG = 5;
  GIVE_ANG = 26;
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);
  SHOOT_UP_TGT = 0;
  SHOOT_DOWN_TGT = 5000;
  LIFT_TGT = -16300;
}

void BALL_Step(void)
{
  ROLL_ANG = 120;
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
  HAL_Delay(3000);

  GIVE_ANG = 80;
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);
  HAL_Delay(1000);

  // BALL_On();
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

  // output[CH2_5] = 0;
  // output[CH1_6] = 0;
  // output[CH1_7] = 0;
  CAN1_cmd_chassis(output[CH1_1], output[CH1_2], output[CH1_3], output[CH1_4], output[CH1_5], output[CH1_6], output[CH1_7], 0);
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_init, 20000, 20);
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_init, 20000, 20);
}

void BALL_Reverse()
{
  ROLL_ANG = 5;
  GIVE_ANG = 26;
  Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
  Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);
  SHOOT_UP_TGT = 0;
  SHOOT_DOWN_TGT = 5000;
  LIFT_TGT = 6300;
}

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void *argument);
void StartBallTask(void *argument);
void StartClampTask(void *argument);
void StartUpperFeedbackTask(void *argument);
void StartSBUSTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

  /* creation of SBUSTask */
  SBUSTaskHandle = osThreadNew(StartSBUSTask, NULL, &SBUSTask_attributes);

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
    if (SWITCH_SBUS_LB_State == SWITCH_MID && SWITCH_LB_State == SWITCH_DOWN || SWITCH_LB_State == SWITCH_MID)
    {
      MAXVAL = 6000;
      Reach_TGT();
      Vx = RC.Vx;
      Vy = RC.Vy;
      omega = RC.omega;
    }
    else if (SWITCH_SBUS_LB_State == SWITCH_UP && SWITCH_LB_State == SWITCH_DOWN)
    {
      // Model controller
      if (SBUS_CH.ConnectState == 0)
      {
        SBUS_LY = 0;
        SBUS_RX = 0;
        SBUS_LX = 0;
      }
      else if (SBUS_CH.ConnectState == 1)
      {
        if (SWITCH_SBUS_RF_State == SWITCH_MID || SWITCH_SBUS_RF_State == SWITCH_UP)
        {
          MAXVAL = 4000;
          SBUS_LY = 10 * (SBUS_CH.CH2 - MR_CH2);
          SBUS_RX = 6 * (SBUS_CH.CH1 - MR_CH1);
          SBUS_LX = 10 * (SBUS_CH.CH4 - ML_CH4);
        }
        else if (SWITCH_SBUS_RF_State == SWITCH_DOWN)
        {
          MAXVAL = 7000;
          SBUS_LY = 17 * (SBUS_CH.CH2 - MR_CH2);
          SBUS_RX = 9 * (SBUS_CH.CH1 - MR_CH1);
          SBUS_LX = 17 * (SBUS_CH.CH4 - ML_CH4);
        }
      }

      Vx = SBUS_LX;
      Vy = SBUS_LY;
      omega = SBUS_RX;

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

      if (fabs(SBUS_CH.CH1 - MR_CH1) < 50)
      {
        omega = 0;
      }
    }
    else if (SWITCH_LB_State == SWITCH_UP)
    {
      // Self-made controller
      if (SWITCH_LF_State == SWITCH_UP)
      {
        MAXVAL = 4000;
        Vx = lx / 9 * 0.75;
        Vy = ly / 9 * 0.75;
        omega = rx / 9 * 0.75; 
      }
      else if (SWITCH_LF_State == SWITCH_DOWN || SWITCH_LF_State == SWITCH_MID)
      {
        MAXVAL = 7000;
        Vx = lx / 9;
        Vy = ly / 9;
        omega = rx / 9;
      }


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

      /* Controller Disconnection Protection */
      factor1[0]++;

      if (receivefactor[0] == 0)
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
    }

    /* Soft Speed UP */
    // V_Sum = sqrt(Vx * Vx + Vy * Vy);
    // if (V_Sum > 3000)
    // {
    //   if (fabs(V_Sum - V_Sum_Last) > 2)
    //   {
    //     V_Sum = V_Sum_Last + 2;
    //   }
    // }
    // Vx = V_Sum * cos(atan2(Vy, Vx));
    // Vy = V_Sum * sin(atan2(Vy, Vx));

    // V_Sum_Last = V_Sum;

    HAL_IWDG_Refresh(&hiwdg);

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
    rtU.yaw_target_CH1_5 = LIFT_TGT;
    rtU.yaw_target_CH1_6 = -SHOOT_DOWN_TGT;
    rtU.yaw_target_CH1_7 = SHOOT_DOWN_TGT;

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
      // R1 Ball Reverse
      BALL_Reverse();
      break;

    default:
      break;
    }

    switch (SWITCH_RF_State)
    {
    case SWITCH_UP:

      GIVE_ANG = GIVE_init;
      ROLL_ANG = 5;
      break;
    case SWITCH_MID:
      GIVE_ANG = GIVE_init;
      ROLL_ANG = ROLL_init;
      break;
    case SWITCH_DOWN:
      GIVE_ANG = 80;
      ROLL_ANG = ROLL_init;
      break;

    default:
      break;
    }
    Set_servo(&htim5, TIM_CHANNEL_1, ROLL_ANG, 20000, 20);
    Set_servo(&htim5, TIM_CHANNEL_2, GIVE_ANG, 20000, 20);

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
    if (SWITCH_RB_State == SWITCH_DOWN)
    {
      __disable_irq();
      NVIC_SystemReset();
    }
    buff_len = sprintf(TransmitBuffer, "fg %d %d %d %d %d %d %d %d\r\n", motor_data[0]->speed_rpm, motor_data[1]->speed_rpm, motor_data[2]->speed_rpm, motor_data[3]->speed_rpm, (int)rtU.yaw_target_CH1_1, (int)rtU.yaw_target_CH1_2, (int)rtU.yaw_target_CH1_3, (int)rtU.yaw_target_CH1_4);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)TransmitBuffer, buff_len);

    osDelay(1);
  }
  /* USER CODE END StartUpperFeedbackTask */
}

/* USER CODE BEGIN Header_StartSBUSTask */
/**
 * @brief Function implementing the SBUSTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSBUSTask */
void StartSBUSTask(void *argument)
{
  /* USER CODE BEGIN StartSBUSTask */
  /* Infinite loop */
  for (;;)
  {
    if (SBUS_CH.CH5 > 1200)
    {
      SWITCH_SBUS_LB_State = SWITCH_DOWN;
    }
    else if (SBUS_CH.CH5 > 800 && SBUS_CH.CH5 < 1200)
    {
      SWITCH_SBUS_LB_State = SWITCH_MID;
    }
    else if (SBUS_CH.CH5 < 800)
    {
      SWITCH_SBUS_LB_State = SWITCH_UP;
    }

    if (SBUS_CH.CH6 > 1200)
    {
      SWITCH_SBUS_RF_State = SWITCH_DOWN;
    }
    else if (SBUS_CH.CH6 > 800 && SBUS_CH.CH6 < 1200)
    {
      SWITCH_SBUS_RF_State = SWITCH_MID;
    }
    else if (SBUS_CH.CH6 < 800)
    {
      SWITCH_SBUS_RF_State = SWITCH_UP;
    }
  osDelay(1);
  }

}
/* USER CODE END StartSBUSTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
