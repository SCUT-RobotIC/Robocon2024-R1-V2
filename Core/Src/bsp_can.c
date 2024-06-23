/**
 *   @brief ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
 **/
#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1; // CAN����1
extern CAN_HandleTypeDef hcan2;

static motor_measure_t motor_chassis[8];
static motor_measure_t motor_chassis1[8];

motor_measure_t *motor_data[8];
motor_measure_t *motor_data1[8];

static CAN_TxHeaderTypeDef chassis_tx_message; // can1前八个
static CAN_TxHeaderTypeDef chassis_tx_message1; // can2前八个
static CAN_TxHeaderTypeDef chassis_tx_message2; // can1后八个
static CAN_TxHeaderTypeDef chassis_tx_message3; // can2后八个

static uint8_t chassis_can_send_data[8]; // can1前八个
static uint8_t chassis_can_send_data1[8]; // can2前八个
static uint8_t chassis_can_send_data2[8]; // can1后八个
static uint8_t chassis_can_send_data3[8]; // can2后八个

/// @brief
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

void circle_cc(motor_measure_t *ptr)
{

  if (((ptr)->ecd - (ptr)->last_ecd) > 5000)
  {
    ptr->circle -= 1;
  }
  else if (((ptr)->ecd - (ptr)->last_ecd) < -5000)
  {

    ptr->circle += 1;
  }
}

/*
������ݣ�      0:���̵��1 3508���,              1:���̵��2 3508���,
                2:���̵��3 3508���,              3:���̵��4 3508���;
                4:yaw��̨��� 6020���;            5:pitch��̨��� 6020���;
                6:������� 2006���
*/

/// @brief hal��CAN�ص�����,���յ������
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    case CAN_3508_M5_ID:
    case CAN_3508_M6_ID:
    case CAN_3508_M7_ID:
    case CAN_3508_M8_ID:
    {

      static uint8_t i = 0;
      // get motor id
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      circle_cc(&motor_chassis[i]);
      break;
    }
    default:
    {
      break;
    }
    }
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }

  if (hcan == &hcan2)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
    case CAN_2006_M1_ID:
    case CAN_2006_M2_ID:
    case CAN_2006_M3_ID:
    case CAN_2006_M4_ID:
    case CAN_2006_M5_ID:
    case CAN_2006_M6_ID:
    case CAN_2006_M7_ID:
    case CAN_2006_M8_ID:
    {

      static uint8_t i = 0;
      // get motor id
      i = rx_header.StdId - CAN_2006_M1_ID;
      get_motor_measure(&motor_chassis1[i], rx_data);
      circle_cc(&motor_chassis1[i]);
      break;
    }
    default:
    {
      break;
    }
    }
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
}

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 */

void CAN1_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

  chassis_tx_message2.StdId = CAN_GIMBAL_ALL_ID;
  chassis_tx_message2.IDE = CAN_ID_STD;
  chassis_tx_message2.RTR = CAN_RTR_DATA;
  chassis_tx_message2.DLC = 0x08;
  chassis_can_send_data2[0] = motor5 >> 8;
  chassis_can_send_data2[1] = motor5;
  chassis_can_send_data2[2] = motor6 >> 8;
  chassis_can_send_data2[3] = motor6;
  chassis_can_send_data2[4] = motor7 >> 8;
  chassis_can_send_data2[5] = motor7;
  chassis_can_send_data2[6] = motor8 >> 8;
  chassis_can_send_data2[7] = motor8;

  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message2, chassis_can_send_data2, &send_mail_box);
}
void CAN2_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
  uint32_t send_mail_box;
  chassis_tx_message1.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message1.IDE = CAN_ID_STD;
  chassis_tx_message1.RTR = CAN_RTR_DATA;
  chassis_tx_message1.DLC = 0x08;
  chassis_can_send_data1[0] = motor1 >> 8;
  chassis_can_send_data1[1] = motor1;
  chassis_can_send_data1[2] = motor2 >> 8;
  chassis_can_send_data1[3] = motor2;
  chassis_can_send_data1[4] = motor3 >> 8;
  chassis_can_send_data1[5] = motor3;
  chassis_can_send_data1[6] = motor4 >> 8;
  chassis_can_send_data1[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message1, chassis_can_send_data1, &send_mail_box);

  chassis_tx_message3.StdId = CAN_GIMBAL_ALL_ID;
  chassis_tx_message3.IDE = CAN_ID_STD;
  chassis_tx_message3.RTR = CAN_RTR_DATA;
  chassis_tx_message3.DLC = 0x08;
  chassis_can_send_data3[0] = motor5 >> 8;
  chassis_can_send_data3[1] = motor5;
  chassis_can_send_data3[2] = motor6 >> 8;
  chassis_can_send_data3[3] = motor6;
  chassis_can_send_data3[4] = motor7 >> 8;
  chassis_can_send_data3[5] = motor7;
  chassis_can_send_data3[6] = motor8 >> 8;
  chassis_can_send_data3[7] = motor8;

  HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message3, chassis_can_send_data3, &send_mail_box);
}

// ���� 3508�������ָ��
motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i)];
}

motor_measure_t *get_chassis_motor_measure_point1(uint8_t i)
{
  return &motor_chassis1[(i)];
}

void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void motor_state_update()
{

  motor_data[0] = get_chassis_motor_measure_point(0);
  motor_data[1] = get_chassis_motor_measure_point(1);
  motor_data[2] = get_chassis_motor_measure_point(2);
  motor_data[3] = get_chassis_motor_measure_point(3);
  motor_data[4] = get_chassis_motor_measure_point(4);
  motor_data[5] = get_chassis_motor_measure_point(5);
  motor_data[6] = get_chassis_motor_measure_point(6);
  motor_data[7] = get_chassis_motor_measure_point(7);
}
void motor_state_update1()
{

  motor_data1[0] = get_chassis_motor_measure_point1(0);
  motor_data1[1] = get_chassis_motor_measure_point1(1);
  motor_data1[2] = get_chassis_motor_measure_point1(2);
  motor_data1[3] = get_chassis_motor_measure_point1(3);
  motor_data1[4] = get_chassis_motor_measure_point1(4);
  motor_data1[5] = get_chassis_motor_measure_point1(5);
  motor_data1[6] = get_chassis_motor_measure_point1(6);
  motor_data1[7] = get_chassis_motor_measure_point1(7);
}

//*******************************************************************�¼�
