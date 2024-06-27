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

uint8_t SWITCH_LF_State = 0;
uint8_t SWITCH_LB_State = 0;
uint8_t SWITCH_RF_State = 0;
uint8_t SWITCH_RB_State = 0;

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
			if ((B1 & 0x01) == 0 && (DataRe.data[BOT1] & 0x01) == 0x01)
			{
				//B1_count[0]++;
				// 按下按键1

				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = 1;
					next_state = 1;
				}
			}
			else if ((B1 & 0x01) == 0x01 && (DataRe.data[BOT1] & 0x01) == 0)
			{
				// 松开按键1
			}

			if ((B1 & 0x02) == 0 && (DataRe.data[BOT1] & 0x02) == 0x02)
			{
				//B1_count[1]++;
				// 按下按键2
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = 1;
					next_state = 2;
				}
			}
			else if ((B1 & 0x02) == 0x02 && (DataRe.data[BOT1] & 0x02) == 0)
			{
				// 松开按键2
			}

			if ((B1 & 0x04) == 0 && (DataRe.data[BOT1] & 0x04) == 0x04)
			{
				//B1_count[2]++;
				// 按下按键
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = 1;
					next_state = 7;
				}
			}
			else if ((B1 & 0x04) == 0x04 && (DataRe.data[BOT1] & 0x04) == 0)
			{
				// 松开按键3
			}

			if ((B1 & 0x08) == 0 && (DataRe.data[BOT1] & 0x08) == 0x08)
			{
				//B1_count[3]++;
				// 按下按键4
			}
			else if ((B1 & 0x08) == 0x08 && (DataRe.data[BOT1] & 0x08) == 0)
			{
				// 松开按键4
			}

			if ((B1 & 0x10) == 0 && (DataRe.data[BOT1] & 0x10) == 0x10)
			{
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = 1;
					next_state = 0;
				}
				//B1_count[4]++;
				// 按下按键5
			}
			else if ((B1 & 0x10) == 0x10 && (DataRe.data[BOT1] & 0x10) == 0)
			{
				// 松开按键5
			}

			if ((B1 & 0x20) == 0 && (DataRe.data[BOT1] & 0x20) == 0x20)
			{
				if (!LOGIC_FLAG)
				{
					LOGIC_FLAG = 1;
					next_state = 7;
				}
				//B1_count[5]++;
				// 按下按键6
			}
			else if ((B1 & 0x20) == 0x20 && (DataRe.data[BOT1] & 0x20) == 0)
			{
				// 松开按键6
			}

			if ((B1 & 0x40) == 0 && (DataRe.data[BOT1] & 0x40) == 0x40)
			{
				//B1_count[6]++;
				// 按下按键7
			}
			else if ((B1 & 0x40) == 0x40 && (DataRe.data[BOT1] & 0x40) == 0)
			{
				// 松开按键7
			}

			if ((B1 & 0x80) == 0 && (DataRe.data[BOT1] & 0x80) == 0x80)
			{
				//B1_count[7]++;
				// 按下按键8
			}
			else if ((B1 & 0x80) == 0x80 && (DataRe.data[BOT1] & 0x80) == 0)
			{
				// 松开按键8
			}

			// 开关更新
			if ((B2 & 0x01) == 0 && (DataRe.data[BOT2] & 0x01) == 0x01)
			{
				// 开关A1开
				SWITCH_LB_State = 1;
				temp_switch = 1;
			}
			else if ((B2 & 0x01) == 0x01 && (DataRe.data[BOT2] & 0x01) == 0)
			{
				// 开关A1关
				SWITCH_LB_State = 0;
				temp_switch = 0;
			}

			if ((B2 & 0x02) == 0 && (DataRe.data[BOT2] & 0x02) == 0x02)
			{
				// 开关A2开
				temp_switch = 2;
			}
			else if ((B2 & 0x02) == 0x02 && (DataRe.data[BOT2] & 0x02) == 0)
			{
				// 开关A2关
				temp_switch = 0;
			}

			if ((B2 & 0x04) == 0 && (DataRe.data[BOT2] & 0x04) == 0x04)
			{
				// 开关B1开
				temp_switch = 3;
			}
			else if ((B2 & 0x04) == 0x04 && (DataRe.data[BOT2] & 0x04) == 0)
			{
				// 开关B1关
				temp_switch = 0;
			}

			if ((B2 & 0x08) == 0 && (DataRe.data[BOT2] & 0x08) == 0x08)
			{
				// 开关B2开
				temp_switch = 4;
			}
			else if ((B2 & 0x08) == 0x08 && (DataRe.data[BOT2] & 0x08) == 0)
			{
				// 开关B2关
				temp_switch = 0;
			}

			if ((B2 & 0x10) == 0 && (DataRe.data[BOT2] & 0x10) == 0x10)
			{

				// 开关C1开
			}
			else if ((B2 & 0x10) == 0x10 && (DataRe.data[BOT2] & 0x10) == 0)
			{
				// 开关C1关
			}

			if ((B2 & 0x20) == 0 && (DataRe.data[BOT2] & 0x20) == 0x20)
			{
				// 开关C2开
			}
			else if ((B2 & 0x20) == 0x20 && (DataRe.data[BOT2] & 0x20) == 0)
			{
				// 开关C2关
			}

			if ((B2 & 0x40) == 0 && (DataRe.data[BOT2] & 0x40) == 0x40)
			{
				// 开关D1开
			}
			else if ((B2 & 0x40) == 0x40 && (DataRe.data[BOT2] & 0x40) == 0)
			{
				// 开关D1关
			}

			if ((B2 & 0x80) == 0 && (DataRe.data[BOT2] & 0x80) == 0x80)
			{
				// 开关D1开
			}
			else if ((B2 & 0x80) == 0x80 && (DataRe.data[BOT2] & 0x80) == 0)
			{
				// 开关D2关
			}
			// 更新标志位

			B1 = DataRe.data[BOT1];
			B2 = DataRe.data[BOT2];
		}
	}
}
