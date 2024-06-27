#ifndef NEW_LOGIC_H
#define NEW_LOGIC_H
#include "main.h"
#include "i2c.h"

#define M_3508 2
#define M_3508_L 3
typedef struct
{
  uint8_t state;
} MotorExtentTypeDef;

typedef void (*func_ptr)();


extern uint8_t HIGH_TROQUE_TRANS_FLAG;
extern MotorExtentTypeDef motorExtent;
extern float ANG_TGT[8];
extern float SPD_TGT[8];
extern float YAW_TGT[8];
extern uint8_t LOGIC_FLAG;
extern uint8_t next_state;
extern uint8_t current_state;
extern uint8_t servo_flag;
extern uint8_t servo_state;

void handle_M_3508_UP(void);
void handle_M_3508_DOWN(void);
void handle_FRONT_CATCH_SERVO_ON(void);
void handle_FRONT_CATCH_SERVO_OFF(void);
void handle_TRANS_ON(void);
void handle_TRANS_OFF(void);
void handle_BACK_SERVO_ON(void);
void handle_BACK_SERVO_OFF(void);
void handle_HIGH_TORQUE(uint8_t *motorExtent);
void LOGIC(void);

void task_yaw_catch(void);
void task_yaw_replace(void);

#endif
