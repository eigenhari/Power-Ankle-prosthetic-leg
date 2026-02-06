#ifndef ROBOT_H_
#define ROBOT_H_

// #include "stm32f4xx.h"

// #include "usart.h"

#ifdef __cplusplus
// #include "definations.hpp"
#include "ak60.hpp"
#include "bno08.hpp"
#include "encoder.hpp"

class Robot
{
public:
    AK60 motor;
    Bno08 imu;
    Encoder encoder;

    
    Robot();
    Robot(UART_HandleTypeDef *, UART_HandleTypeDef *);
};

#endif

#ifdef __cplusplus
extern "C"
{
#endif
    void init_robot();
    void operate_robot();
#ifdef __cplusplus
}
#endif

#endif // ROBOT_H_
