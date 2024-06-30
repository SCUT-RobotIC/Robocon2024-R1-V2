#include "solve.h" // Device header

extern TGT_COOR TC;
extern REAL_COOR RC;
extern uint8_t USART2_RX_BUF[100];
extern motor_measure_t *motor_data[8];
uint8_t data[10];

double factors1 = 2;
double factors2 = 30;
double por = 2.5;

double deadband = 50;
double top = 4000;
double a1, a2, a3, a4, a5, a6;

void Receive()
{
  RC.x = (USART2_RX_BUF[2] << 8) | USART2_RX_BUF[1];
  RC.y = (USART2_RX_BUF[4] << 8) | USART2_RX_BUF[3];
  RC.theta = (USART2_RX_BUF[6] << 8) | USART2_RX_BUF[5];

  RC.xll = RC.xlast;
  RC.yll = RC.ylast;

  RC.xlast = RC.x;
  RC.ylast = RC.y;

  TC.x = (USART2_RX_BUF[8] << 8) | USART2_RX_BUF[7];
  TC.y = (USART2_RX_BUF[10] << 8) | USART2_RX_BUF[9];
  TC.theta = (USART2_RX_BUF[12] << 8) | USART2_RX_BUF[11];

  RC.action = (USART2_RX_BUF[14] << 8) | USART2_RX_BUF[13];
}
void Reach_TGT()
{
  RC.distll = RC.distlast;
  RC.distlast = RC.dist;
  RC.dist = sqrt(pow(TC.y - RC.y, 2) + pow((TC.x - RC.x), 2));
  TC.XYtheta = atan2(TC.y - RC.y, TC.x - RC.x) * 180 / PI;

  if (fabs((double)RC.dist) >= deadband && fabs((double)RC.dist) < 1000)
  {
    factors1 = 2;
    RC.dist = 1000 / factors1;
  }
  else if (fabs((double)RC.dist) > 1000)
  {
    factors1 = 4;
  }
  if (fabs((double)RC.dist) > top)
  {

    RC.dist = top;
  }

  RC.RE_theta = TC.XYtheta - RC.theta;
  if (fabs((double)RC.RE_theta) < 1)
  {
    RC.RE_theta = 0;
  }
  if (fabs((double)RC.dist) < deadband)
  // if (fabs((double)RC.dist) < deadband || fabs(TC.theta - RC.theta) > 10)
  {
    RC.Vx = 0;
    RC.Vy = 0;
  }
  else
  {
    RC.Vx = ((RC.dist + (RC.dist - RC.distlast) * por) * cos(RC.RE_theta * PI / 180)) * factors1;
    RC.Vy = ((RC.dist + (RC.dist - RC.distlast) * por) * sin(RC.RE_theta * PI / 180)) * factors1;
  }

  if (TC.theta - RC.theta > 181)
    TC.theta = TC.theta - 360;
  if (TC.theta - RC.theta < -181)
    TC.theta = TC.theta + 360;
  TC.theta = TC.theta % 360;

  if (fabs((double)TC.theta - RC.theta) > 3)
    RC.omega = (TC.theta - RC.theta) * factors2;
  else
    RC.omega = 0;

  if (fabs((double)RC.omega) > 3000)
  {
    if (RC.omega > 0)
      RC.omega = 3000;
    else
      RC.omega = -3000;
  }
  else if (fabs((double)RC.omega) < 650)
  {
    if (RC.omega > 0)
      RC.omega = 650;
    else if (RC.omega < 0)
      RC.omega = -650;
    else
      RC.omega = 0;
  }
  RC.omega = -RC.omega;
} // 最后把RC.Vx，RC.Vy，RC.omege三个参数放进main接收就行

void CAL_TXMESSAGE(void)
{

  data[0] = 0x0F;
  for (int i = 0; i < 4; i++)
  {

    data[2 * i + 1] = motor_data[i]->speed_rpm >> 8;
    data[2 * i + 2] = motor_data[i]->speed_rpm;
  }
  data[9] = 0xAA;
}
