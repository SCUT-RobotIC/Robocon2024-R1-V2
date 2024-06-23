#include "new_logic.h"
#include "i2c.h"
///
void handle_M_3508_UP(void){
    //ANG_TAG
    YAW_TGT[M_3508] = 720;
}

void handle_M_3508_DOWN(void){
    //ANG_TAG
    YAW_TGT[M_3508] = 120;
}

void handle_FRONT_CATCH_SERVO_ON(void){
    //GPIO控制
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}

void handle_FRONT_CATCH_SERVO_OFF(void){
    //GPIO控制
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}

void handle_FRONT_LEN_SERVO_ON(void){
    //GPIO控制
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

void handle_FRONT_LEN_SERVO_OFF(void){
    //GPIO控制
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

void handle_TRANS_ON(void){
    //SPD_TAG
    YAW_TGT[M_2006]=360;
}

void handle_TRANS_OFF(void){
    //SPD_TAG
    YAW_TGT[M_2006]=0;
}

void handle_BACK_SERVO_ON(void){
    //PWM
}

void handle_BACK_SERVO_OFF(void){
    //PWM
}

void handle_PLACE_SERVO_ON(void){
    //PWM
}

void handle_PLACE_SERVO_OFF(void){
    //PWM
}

void handle_HIGH_TORQUE(uint8_t *motorExtent){
    //I2C_TRANS
    
}

void task_yaw_catch(void){
    YAW_TGT[M_3508] = 1000;
    HAL_Delay(4000);
    motorExtent.state = 0xcd;
    HIGH_TROQUE_TRANS_FLAG=1;
    HAL_Delay(4000);
    motorExtent.state = 0xab;
    HIGH_TROQUE_TRANS_FLAG =1;
    HAL_Delay(4000);
    YAW_TGT[M_3508] = 120;
}

void task_yaw_replace(void){
    
}

void init(void){
    motorExtent.state = 0xab;
    HIGH_TROQUE_TRANS_FLAG=1;
    HAL_Delay(1000);
    YAW_TGT[M_3508] = 120;
    HAL_Delay(1000);
}

void close(void){

}

func_ptr func_table[] = {
    init,
    task_yaw_catch,
    task_yaw_replace,
    handle_TRANS_ON,
    handle_TRANS_OFF,
    handle_PLACE_SERVO_ON,
    handle_PLACE_SERVO_OFF,
    close
};


void LOGIC(void){
    if(LOGIC_FLAG){
        //if(current_state != next_state){
            func_table[next_state]();
            current_state = next_state;
        //}
        LOGIC_FLAG=0;
    }
}


