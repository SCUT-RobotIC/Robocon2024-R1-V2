#include "new_logic.h"
#include "i2c.h"


void claw_catch(void)
{
    claw_flag = on;
    claw_state = state_claw_catch;
    // HAL_Delay(200);
    // YAW_TGT[M_3508_R] = 400;
    // YAW_TGT[M_3508_L] = -400;
    HAL_Delay(300);
    YAW_TGT[M_3508_R] = 450;
    YAW_TGT[M_3508_L] = -450;
    next_place = FST_PLACE;
}

void claw_place(void)
{
    switch (next_place){
        case FST_PLACE:
            YAW_TGT[M_3508_R] = 0;
            YAW_TGT[M_3508_L] = 0;
            HAL_Delay(500);
            claw_flag = on;
            claw_state = state_claw_place;
            HAL_Delay(200);
            YAW_TGT[M_3508_R] = 200;
            YAW_TGT[M_3508_L] = -200;
            //next_place = SEC_PLACE;
        break;
        case SEC_PLACE:
            YAW_TGT[M_3508_R] = 0;
            YAW_TGT[M_3508_L] = 0;
            HAL_Delay(500);
            claw_flag = on;
            claw_state = state_claw_place;
            //next_place = IDLE_PLACE;
        break;
        default:
        break;
    }
}

void init(void)
{
    motorExtent.state = 0xab;
    HIGH_TROQUE_TRANS_FLAG = on;
    HAL_Delay(500);
    YAW_TGT[M_3508_R] = 0;
    YAW_TGT[M_3508_L] = 0;
    claw_flag = on;
    claw_state = state_init;
    
}

void close(void)
{
    claw_flag = on;
    claw_state = state_close;
    motorExtent.state = 0xcd;
    HIGH_TROQUE_TRANS_FLAG = on;
    HAL_Delay(500);
    YAW_TGT[M_3508_R] = 0; 
    YAW_TGT[M_3508_L] = 0;
}

func_ptr func_table[] = {
    init,
    close,
    claw_catch,
    claw_place
};

void LOGIC(void)
{
    if (LOGIC_FLAG && next_state!=IDLE_CLAW)
    {
        func_table[next_state]();
        LOGIC_FLAG = off;
    }
}
