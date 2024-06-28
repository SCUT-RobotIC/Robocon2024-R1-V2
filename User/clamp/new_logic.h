#ifndef NEW_LOGIC_H
#define NEW_LOGIC_H
#include "main.h"
#include "i2c.h"

#define M_3508_R 2
#define M_3508_L 3

#define STR_ON            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#define STR_OFF           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
#define CLAW_CH1_CH3_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
#define CLAW_CH1_CH3_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
#define CLAW_CH2_CH4_ON   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
#define CLAW_CH2_CH4_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

typedef struct
{
  uint8_t state;
} MotorExtentTypeDef;


typedef enum {
  state_init = 0,
  state_close = 1,
  state_claw_catch = 2,
  state_claw_place = 3,
  IDLE_CLAW = 4
}claw_enum;  

typedef enum {
  IDLE_PLACE = 0,
  FST_PLACE = 1,
  SEC_PLACE = 2
}place_enum;


typedef enum {
  off = 0,
  on = 1,
}bool_T;  

typedef void (*func_ptr)();

extern bool_T HIGH_TROQUE_TRANS_FLAG;
extern MotorExtentTypeDef motorExtent;
extern float YAW_TGT[8];
extern float ANG_TGT[8];
extern bool_T LOGIC_FLAG;
extern claw_enum next_state;
extern claw_enum current_state;
extern bool_T claw_flag;
extern claw_enum claw_state;
extern place_enum next_place;

void LOGIC(void);

void claw_catch(void);
void claw_place(void);
void init(void);
void close(void);

#endif
