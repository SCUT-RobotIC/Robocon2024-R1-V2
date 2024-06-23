#ifndef __SOLVE_H
#define __SOLVE_H
#include "main.h" // Device header
#include "arm_math.h"
#include "motorctrl.h"

/* ACTION DEFINITION */
#define CLAMP_PINCH 2

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t theta;
	int16_t XYtheta;
} TGT_COOR;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t theta;

	int16_t xlast;
	int16_t ylast;

	int16_t RE_theta;
	int16_t dist;
	int16_t Vx;
	int16_t Vy;
	int16_t omega;

	// 动作数据位
	int16_t action;
} REAL_COOR;
void Receive(void);
void Reach_TGT(void);
void CAL_TXMESSAGE(void);
#endif
