#include "robot.h"
#include <stdio.h>
#include "gpio.h"
#include "tim.h"
#include "math.h"
#include "printf_config.c"
#include <cstring>


// #define theta 60.0
// #define gear_reduction 4.6;
// #define omega 6.28;

// void send_motor_command(float theta_motor) {
//     char cmd[50];
//     sprintf(cmd, "M %f\n", theta_motor);  
//     HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 100);
// }


Robot::Robot(UART_HandleTypeDef *mhuart, UART_HandleTypeDef *ihuart) : motor(mhuart), imu(ihuart) {};
// PID mot_pid(0.5, 0, 0, 0.5, 5, 10);

Robot bot(&huart2, &huart4);

uint8_t test_data[100];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);
    if (huart->Instance == huart2.Instance) // bot.motor.uart.get_uart_instance())
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        bot.motor.uart.process_receive_callback();
    }
    else if (huart->Instance == huart4.Instance)
    {
        bot.imu.rx_callback();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == bot.motor.uart.get_uart_instance())
    {
        // printf("transmitted\n");
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    }
}

float angle = 0.0;
void init_robot()
{
    bot.motor.init();
    bot.encoder = Encoder(&htim4, 4000, 10);
    bot.encoder.init();
    bot.imu.init();

    // bot.motor.set_position_speed(0, 5000, 10000);
    // // // bot.motor.set_position_speed(90, 1000.0, 5000);
    // HAL_Delay(1000);
    // // mot_pid.init();
    // bot.motor.request_parameters();

    // HAL_Delay(10);
    // bot.motor.set_position_speed(0, 0, 0);

    // // bot.motor.set_position_speed(90, 1000.0, 5000);
    // HAL_Delay(10);

    // bot.motor.set_speed(1000.0);
}

const float position[] = {-30, 30, 60, 0};  // Position values
const float velocity[] = {5000.0, 50000.0, 5000.0, 5000.0};  // Velocity values
const float acceleration[] = {300000, 300000, 300000, 300000};  // Acceleration values



enum GaitPhase
{
    // TO_20_DEG,
    // FROM_20_DEG
    CONTROLLED_PLANTARFLEXION,
    CONTROLLED_DORSIFLEXTION,
    POWERED_PLANTARFLEXION,
    SWING_PHASE
};

// TargetAngleState angle_state = TO_20_DEG;
GaitPhase current_phase =   CONTROLLED_PLANTARFLEXION;
// uint32_t angle_state_change_tick = 0;
uint32_t phase_change_tick =0;

static uint32_t loop_tick = 0;
uint32_t start_time = HAL_GetTick();
void operate_robot()
{
    uint32_t last_tick = 0;
    uint32_t last_led = 0;
    uint32_t callback_tick = 0;    

    while (1)
    {
        if ((HAL_GetTick() - last_tick) < 10)
            continue;
        uint32_t current_time = HAL_GetTick();
        printf("[%lu ms] yprxyz: %f %f %f %f %f %f\n", 
            current_time, 
            bot.imu.get_data().yaw, 
            bot.imu.get_data().pitch, 
            bot.imu.get_data().roll, 
            bot.imu.get_data().accel_x, 
            bot.imu.get_data().accel_y, 
            bot.imu.get_data().accel_z);

        

        printf("[%lu ms] count : %ld %f\n",
           current_time,
           bot.encoder.get_count(),
           bot.encoder.get_omega());
        
        
        // printf("yprxyz: %f %f %f %f %f %f\n", bot.imu.get_data().yaw, bot.imu.get_data().pitch, bot.imu.get_data().roll, bot.imu.get_data().accel_x, bot.imu.get_data().accel_y, bot.imu.get_data().accel_z);

        last_tick = HAL_GetTick();

        // printf("count: %d, omega : %f\n", bot.encoder.get_count(), bot.encoder.get_omega());

        // HAL_UART_Receive_DMA(&huart2, test_data, 100);
        // bot.motor.uart.print_receive_buffer();
        //  if (bot.motor.uart.data_ready)
        
        //   {
        //     bot.motor.update_parameters();
        //     bot.motor.display_parameters();
        //     bot.motor.request_parameters();
        //     HAL_Delay(100);
        // // //     // last_tick = HAL_GetTick();
        //   }

        // if (bot.motor.uart.data_ready)
        // { 
            
        //     bot.motor.update_parameters();
        //     bot.motor.display_parameters();

            
        //     static uint32_t last_request_time = 0;
        //     uint32_t current_time = HAL_GetTick();

        //     if (current_time - last_request_time >= 100) 
        //     {
        //         bot.motor.request_parameters();
        //         last_request_time = current_time;  
        //     }
        // }

        
        // int step = 20;
        // int n um_steps = 10;
        // int current_position = 0;
        // bot.motor.display_parameters();

        // bot.motor.set_position(90.0);
        // bot.motor.uart.print_transmit_buffer();
        // HAL_Delay(10000);
        // bot.motor.set_position(180.0);
        // bot.motor.uart.print_transmit_buffer();
        // HAL_Delay(10000);
        // bot.motor.set_position(0.0);
        // bot.motor.uart.p rint_transmit_buffer();
        // HAL_Delay(10000);
        // bot.motor.set_position_speed(90, 5000.0, 10000);
        // HAL_Delay(5000);
        // bot.motor.set_position_speed(0, 5000.0, 10000);
        // HAL_Delay(5000);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        // switch (angle_state)
        // {
        // case TO_20_DEG:
        //     break;

        // case FROM_20_DEG:
        //     break;
        // }

        // bot.motor.set_position_speed(-40, 4000.0, 30000);
        // // HAL_Delay(1000);
        // bot.motor.set_position_speed(0, 1000.0, 5000);
        // HAL_Delay(1000);

        // if ((HAL_GetTick() - angle_state_change_tick) > 4000)
        // {

        //     switch (angle_state)
        //     {
        //     case TO_20_DEG:
        //         angle = 0;
        //         angle_state = FROM_20_DEG;
        //         break;

        //     case FROM_20_DEG:
        //         angle = 20;
        //         angle_state = TO_20_DEG;
        //         break;
        //     }
        //     angle_state_change_tick = HAL_GetTick();
        // }
        // if((HAL_GetTick()-phase_change_tick)>400)
        // {
        //     switch(current_phase)
        //     {
        //         case CONTROLLED_PLANTARFLEXION:
                
        //             bot.motor.set_position_speed(-15, 5000.0, 10000);
        //             current_phase = CONTROLLED_DORSIFLEXTION;
        //             break;
        //         case  CONTROLLED_DORSIFLEXTION:
        //             bot.motor.set_position_speed(15,5000.0,10000);
        //             current_phase = POWERED_PLANTARFLEXION;
        //             break;
        //         case POWERED_PLANTARFLEXION:
        //             bot.motor.set_position_speed(25,5000.0,10000);
        //             current_phase =  SWING_PHASE;
        //             break;
        //         case SWING_PHASE:
        //             bot.motor.set_position_speed(0, 5000.0, 10000);
        //             current_phase = CONTROLLED_PLANTARFLEXION;
        //             break;
        //     }
        //     phase_change_tick = HAL_GetTick();

        // }


        if ((HAL_GetTick() - phase_change_tick) > 4000) {
            bot.motor.set_position_speed(position[current_phase], 
                                         velocity[current_phase], 
                                         acceleration[current_phase]);
            
            
            current_phase = static_cast<GaitPhase>((current_phase + 1) % 4);
            
            phase_change_tick = HAL_GetTick();
        }




        // bot.motor.set_position_speed(angle, 2000.0, 30000);
        // bot.motor.set_current(0.5);

        // bot.motor.request_parameters();
        // HAL_Delay(50);
        // bot.motor.update_parameters();
        // bot.motor.display_parameters();

        // bot.motor.set_position_speed(-60,5000.0,30000);

        // bot.motor.set_position_speed(0, 1000.0, 5000);
        // HAL_Delay(4000);

        // bot.motor.set_position_speed(270, 1000.0, 5000);
        // HAL_Delay(5000);

        // bot.motor.set_position_speed(90, 1000.0, 5000);
        // HAL_Delay(15000);
        // bot.motor.uart.print_transmit_buffer();
        // HAL_Delay(10000);

        // bot.motor.set_speed(1000.0);
        // bot.motor.uart.print_transmit_buffer();
        // HAL_Delay(10000);
        // bot.motor.request_parameters();

        // uint8_t trans_buffer[] = {0x02, 0x05, 0x08, 0x00, 0x00, 0x03, 0xE8, 0x2B, 0x58, 0x03};

        // HAL_UART_Transmit(&huart2, trans_buffer, 10, 100);
        // HAL_UART_Transmit_DMA(&huart2, trans_buffer, 10);

        // bot.motor.set_speed(3000.0);

        // float elapsed_time = (HAL_GetTick()-start_time)/1000.0;
        // float angle = theta_0*sin(omega*elapsed_time);
        // printf("Time:%f,Angle:%f\n",elapsed_time,angle);
        // bot.motor.set_position_speed(angle,2000.0,50000);
        // bot.motor.set_position_speed(30, 2000.0, 50000);
        // HAL_Delay(1000);
        // // bot.motor.set_position_speed(-30, 2000.0, 50000);
        // HAL_Delay(1000);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart4.Instance)
    {
        bot.imu.init();
    }
}