#include <ak60.hpp>
#include <stdio.h>
#include "buffer.h"

AK60::AK60(UART_HandleTypeDef *huart) : uart(huart) {}

void AK60::init()
{
    uart.init();
    // set_home_position();
    HAL_Delay(100);
}
void AK60::request_parameters()
{
    transmit_data_frame[0] = 0x04;
    if (!uart.transmit(transmit_data_frame, 1))
    {
        // printf("get_motor_parameters could not be tramsmitted.\n");
    }
}

void AK60::update_parameters()
{
    if (!uart.data_ready)
    {
        return;
    }
    uart.get_received_data(receive_data_frame);
    int32_t index = 0;
    // parameters.mos_temp = (float)buffer_get_int16(receive_data_frame, &index) / 10.0;
    // parameters.motor_temp = (float)buffer_get_int16(receive_data_frame, &index) / 10.0;
    parameters.output_current = (float)buffer_get_int32(receive_data_frame, &index) / 100.0;
    parameters.input_current = (float)buffer_get_int32(receive_data_frame, &index) / 100.0;
    parameters.id_current = (float)buffer_get_int32(receive_data_frame, &index) / 100.0;
    parameters.iq_current = (float)buffer_get_int32(receive_data_frame, &index) / 100.0;
    // parameters.throttle = (float)buffer_get_int16(receive_data_frame, &index) / 1000.0;
    // parameters.speed = (float)buffer_get_int32(receive_data_frame, &index);
    parameters.input_voltage = (float)buffer_get_int16(receive_data_frame, &index) / 10.0;
    // index += 24;
    // parameters.outer_loop_position = (float)buffer_get_int32(receive_data_frame, &index) / 1000000.0;
    // parameters.id = receive_data_frame[index];
    // index += 6;
    // parameters.vd_voltage = (float)buffer_get_int32(receive_data_frame, &index) / 1000.0;
    // parameters.vq_voltage = (float)buffer_get_int32(receive_data_frame, &index) / 1000.0;
}

void AK60::display_parameters()
{
    // printf("Displaying motor parameters\n");
    // printf("mos_temp :: %f\n", parameters.mos_temp);
    // printf("motor_temp :: %f\n", parameters.motor_temp);
    printf("output_current :: %f\n", parameters.output_current);
    printf("input_current :: %f\n", parameters.input_current);
    printf("id_current :: %f\n", parameters.id_current);
    printf("iq_current :: %f\n", parameters.iq_current);
    // printf("throttle :: %f\n", parameters.throttle);
    printf("speed :: %f\n", parameters.speed);
    printf("input_voltage :: %f\n", parameters.input_voltage);
}

void AK60::set_position(float position)
{
    uint8_t pos[4];
    int32_t index = 0;
    transmit_data_frame[index++] = 0x09;
    buffer_append_float32(transmit_data_frame, position, 100000.0, &index);
    if (!uart.transmit(transmit_data_frame, index))
    {
        // printf("set_position command successfully transmitted.\n");
    }
}

void AK60::set_home_position()
{
    transmit_data_frame[0] = 0x5F;
    transmit_data_frame[1] = 0x01;
    if (!uart.transmit(transmit_data_frame, 2))
    {
        // printf("set_sero_position command successfully transmitted.\n");
    }
}

void AK60::set_speed(float speed)
{
    uint8_t pos[4];
    int32_t index = 0;
    transmit_data_frame[index++] = 0x08;
    buffer_append_float32(transmit_data_frame, speed, 1.0, &index);
    if (!uart.transmit(transmit_data_frame, index))
    {
        // printf("set_speed command successfully transmitted.\n");
    }
}

void AK60::set_position_speed(float position, float speed, float acc)
{
    uint8_t pos[4];
    int32_t index = 0;
    transmit_data_frame[index++] = 0x5B;
    

    buffer_append_float32(transmit_data_frame, position, 1000.0, &index);
    buffer_append_float32(transmit_data_frame, speed, 1.0, &index);
    buffer_append_float32(transmit_data_frame, acc, 1.0, &index);


    if (!uart.transmit(transmit_data_frame, index))
    {
        // printf("set_position_speed command successfully transmitted.\n");
    }
}

void AK60::set_current(float current)
{
    uint8_t curr[4];
    int32_t index = 0;
    transmit_data_frame[index++] = 0x06;
    buffer_append_float32(transmit_data_frame, current, 1000.0, &index);
    if (!uart.transmit(transmit_data_frame, index))
    {
        // printf("set_position command successfully transmitted.\n");
    }
}
