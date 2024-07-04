/****
	*   @brief 本代码是遥控器接收端的解析代码
	*   @brief 使用方法：
	*   @brief 一、将下列代码复制到main函数的全局变量声明中，然后就可以在main函数中使用这些数据

	DataPacket DataRe;
	int16_t lx,ly,rx,ry,lp,rp;
	uint8_t B1,B2;
	uint8_t Cal_Parity;
	uint8_t USART_FLAG=0;

	*   @brief 二、在初始化部分启动一次DMA接收

	HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, 1);

	*   @brief 三、将接收代码复制到main函数的用户代码部分

	//接收回调
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		if (huart->Instance == USART3)
		{
			if(DataRe.header==0xAA && USART_FLAG==0)
			{
				CAL_MESSAGE();
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe+1, sizeof(DataPacket)-1);
				USART_FLAG=1;
			}
			if(DataRe.header==0xAA && USART_FLAG==1)
			{
				CAL_MESSAGE();
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, sizeof(DataPacket));
			}
			else
			{
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, 1);
				USART_FLAG=0;
			}
		}
	}

	//空闲回调
	void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
	{
		if (huart->Instance == USART3)
		{
			HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, 1);
			USART_FLAG=0;
		}
	}

	//错误回调
	void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
	{
		if (huart->Instance == USART3)
		{
			HAL_UART_Receive_DMA(&huart3, (uint8_t*)&DataRe, 1);
			USART_FLAG=0;
		}
	}

	*
	****/
#include "calculate.h"
#include "main.h"

switch_enum SWITCH_LF_State = SWITCH_UP;
switch_enum SWITCH_LB_State = SWITCH_UP;
switch_enum SWITCH_RF_State = SWITCH_MID;
switch_enum SWITCH_RB_State = SWITCH_UP;

uint8_t BUTTON_State = 0;

// SBUS左上拨杆开关状态
switch_enum SWITCH_SBUS_LB_State = SWITCH_UP;
// SBUS右上拨杆开关状态
switch_enum SWITCH_SBUS_RF_State = SWITCH_UP;

int16_t temp_switch = 0;

// 计算校验位
uint8_t CalculateParity(const uint8_t *data, int dataSize)
{
	// 奇校验
	uint8_t parity = 1;
	for (int i = 0; i < dataSize; i++)
	{
		parity ^= data[i];
	}
	return parity;
}

void CAL_MESSAGE(void)
{
	// 检测帧头帧尾是否正确
	if (DataRe.header == 0xAA && DataRe.footer == 0xDD)
	{
		Cal_Parity = CalculateParity(DataRe.data, sizeof(DataRe.data));
		// 判断奇偶校验是否合规
		if (Cal_Parity == DataRe.parity)
		{
			// 解算摇杆与电位器数据
			lx = (DataRe.data[LX_MSB] << 8) | DataRe.data[LX_LSB];
			ly = (DataRe.data[LY_MSB] << 8) | DataRe.data[LY_LSB];
			rx = (DataRe.data[RX_MSB] << 8) | DataRe.data[RX_LSB];
			ry = (DataRe.data[RY_MSB] << 8) | DataRe.data[RY_LSB];
			lp = (DataRe.data[LP_MSB] << 8) | DataRe.data[LP_LSB];
			rp = (DataRe.data[RP_MSB] << 8) | DataRe.data[RP_LSB];
			// 符号扩展
			lx = (lx & 0x8000) ? (lx | 0xFFFF0000) : lx;
			ly = (ly & 0x8000) ? (ly | 0xFFFF0000) : ly;
			rx = (rx & 0x8000) ? (rx | 0xFFFF0000) : rx;
			ry = (ry & 0x8000) ? (ry | 0xFFFF0000) : ry;
			lp = (lp & 0x8000) ? (lp | 0xFFFF0000) : lp;
			rp = (rp & 0x8000) ? (rp | 0xFFFF0000) : rp;

			// 按键更新
			if ((B1 & 0x01) == 0 && (DataRe.data[BOT1]) == 0x01)
			{
				B1_count[0]++;
				// 按下按键1
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = on;
					next_state = state_claw_catch;
				}
			}

			if ((B1 & 0x02) == 0 && (DataRe.data[BOT1]) == 0x02)
			{
				B1_count[1]++;
				// 按下按键2
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = on;
					next_state = state_claw_place;
				}
			}

			if ((B1 & 0x04) == 0 && (DataRe.data[BOT1]) == 0x04)
			{
				B1_count[2]++;
				// 按下按键3
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = on;
					next_state = state_init;
				}
			}

			if ((B1 & 0x08) == 0 && (DataRe.data[BOT1]) == 0x08)
			{
				B1_count[3]++;
				// 按下按键4
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = on;
					next_state = state_close;
				}
			}

			if ((B1 & 0x10) == 0 && (DataRe.data[BOT1]) == 0x10)
			{
				B1_count[4]++;
				// 按下按键5, 球发射暂停
				BUTTON_State = 1;
			}

			if ((B1 & 0x20) == 0 && (DataRe.data[BOT1]) == 0x20)
			{
				B1_count[5]++;
				// 按下按键6，球发射上膛
				BUTTON_State = 2;
			}

			if ((B1 & 0x40) == 0 && (DataRe.data[BOT1]) == 0x40)
			{
				B1_count[6]++;
				BUTTON_State = 3;
			}

			if ((B1 & 0x80) == 0 && (DataRe.data[BOT1]) == 0x80)
			{
				B1_count[7]++;
				// 按下按键8
			}

			// 开关更新
			if ((B2 & 0x03) == 0x01)
			{
				// 开关A1开
				SWITCH_LB_State = SWITCH_DOWN;
			}
			if ((B2 & 0x03) == 0x03)
			{
				// 开关A1关
				SWITCH_LB_State = SWITCH_MID;
			}
			if ((B2 & 0x03) == 0x02)
			{
				// 开关A2开
				SWITCH_LB_State = SWITCH_UP;
			}

			if ((B2 & 0x0C) == 0x04)
			{
				// 开关B1开
				SWITCH_LF_State = SWITCH_UP;
			}
			if ((B2 & 0x0C) == 0x0C)
			{
				// 开关B关
				SWITCH_LF_State = SWITCH_MID;
			}
			if ((B2 & 0x0C) == 0x08)
			{
				// 开关B2开
				SWITCH_LF_State = SWITCH_DOWN;
			}

			if ((B2 & 0x30) == 0x10)
			{
				// 开关C1开
				SWITCH_RB_State = SWITCH_UP;
			}
			if ((B2 & 0x30) == 0x30)
			{
				// 开关C1关
				SWITCH_RB_State = SWITCH_MID;
			}
			if ((B2 & 0x30) == 0x20)
			{
				// 开关C2开
				SWITCH_RB_State = SWITCH_DOWN;
			}

			//			if ((B2 & 0xC0) == 0xC0 && (DataRe.data[BOT2] & 0xC0) == 0x40)
			//			{
			//				// 开关D1开
			//				SWITCH_RF_State = 1;
			//			}
			//			if (((B2 & 0xC0) == 0x80 && (DataRe.data[BOT2] & 0xC0) == 0xC0)||((B2 & 0xC0) == 0x40 && (DataRe.data[BOT2] & 0xC0) == 0xC0))
			//			{
			//				// 开关D1关
			//				SWITCH_RF_State = 2;
			//			}
			//			if ((B2 & 0xC0) == 0xC0 && (DataRe.data[BOT2] & 0xC0) == 0x80)
			//			{
			//				// 开关D1开
			//				SWITCH_RF_State = 3;
			//			}

			if ((B2 & 0xC0) == 0x40)
			{
				// 开关D1开
				SWITCH_RF_State = SWITCH_UP;
			}
			if ((B2 & 0xC0) == 0xC0)
			{
				// 开关D1关
				SWITCH_RF_State = SWITCH_MID;
			}
			if ((B2 & 0xC0) == 0x80)
			{
				// 开关D1开
				SWITCH_RF_State = SWITCH_DOWN;
			}

			// 更新标志位

			B1 = DataRe.data[BOT1];
			B2 = DataRe.data[BOT2];
		}
	}
}
