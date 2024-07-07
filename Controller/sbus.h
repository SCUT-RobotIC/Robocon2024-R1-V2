
#ifndef __SBUS_H
#define __SBUS_H

#include "stm32f4xx_hal.h"

#define SBUS_FRAME_SIZE 25
#define SBUS_INPUT_CHANNELS 16

// ֨ӥsubsхۅքخСֵ خճֵ אֵ ̀ȸ ӔܰϣλתۻԉPWMֵք׶Χè1000-2000é
#define SBUS_RANGE_MIN 300.0f
#define SBUS_RANGE_MAX 1700.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define DEAD_RANGE_MIN 960 // ̀ȸ
#define DEAD_RANGE_MAX 1040
#define SBUS_RANGE_MIDDLE 1000.0f
#define SBUS_CONNECT_FLAG 0x00

// ֍̙ԫٟ̙ģʽì֢oԃһٶ׾׎ߪژ࠘׆̙׈յλ
#define LOW_SPEED 0
#define HIGH_SPEED 1

// ֨ӥ̄ٶҡًԫҦ֯ߪژք٦Ŝ
#define YAW 1
#define THROTTLE 2
#define PITCH 3
#define ROLL 4
#define SPEED_MODE 6

#define ML_CH3 1224 // left_spd
#define ML_CH4 1000 // rotate
#define MR_CH2 1000	 //  right_y  max 1400 min 600
#define MR_CH1 1000	 // right_x

extern int command[20]; // ң࠘Ƿ˽ߝ

typedef struct
{
	uint16_t signal[25];
	uint16_t CH1;		  // ͨր1˽ֵ
	uint16_t CH2;		  // ͨր2˽ֵ
	uint16_t CH3;		  // ͨր3˽ֵ
	uint16_t CH4;		  // ͨր4˽ֵ
	uint16_t CH5;		  // ͨր5˽ֵ
	uint16_t CH6;		  // ͨր6˽ֵ
	uint16_t CH7;		  // ͨր7˽ֵ
	uint16_t CH8;		  // ͨր8˽ֵ
	uint16_t CH9;		  // ͨր9˽ֵ
	uint16_t CH10;		  // ͨր10˽ֵ
	uint16_t CH11;		  // ͨր11˽ֵ
	uint16_t CH12;		  // ͨր12˽ֵ
	uint16_t CH13;		  // ͨր13˽ֵ
	uint16_t CH14;		  // ͨր14˽ֵ
	uint16_t CH15;		  // ͨր15˽ֵ
	uint16_t CH16;		  // ͨր16˽ֵ
	uint8_t ConnectState; // ң࠘Ƿԫޓ˕Ƿlޓ״̬ 0=δlޓì1=ֽӣlޓ
} SBUS_CH_Struct;

// 上中下三档拨杆开关结构体
typedef enum
{
  SWITCH_UP = 0,
  SWITCH_MID = 1,
  SWITCH_DOWN = 2
} switch_enum;

extern SBUS_CH_Struct SBUS_CH;
extern int SBUS_LY, SBUS_LX, SBUS_RX;

extern switch_enum SBUS_SWA_State;
extern switch_enum SBUS_SWB_State;
extern switch_enum SBUS_SWC_State;
extern switch_enum SBUS_SWD_State;

extern switch_enum SBUS_SWA_Last_State;
extern switch_enum SBUS_SWB_Last_State;
extern switch_enum SBUS_SWC_Last_State;
extern switch_enum SBUS_SWD_Last_State;

// SBUSхۅޢ϶Рژگ˽
uint8_t update_sbus(uint8_t *buf);
uint16_t sbus_to_pwm(uint16_t sbus_value);
float sbus_to_Range(uint16_t sbus_value, float p_min, float p_max);
#endif
