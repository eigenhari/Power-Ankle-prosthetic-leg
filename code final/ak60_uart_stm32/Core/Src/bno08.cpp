#include <stdio.h>
#include <memory.h>
#include "bno08.hpp"

bool Bno08::init()
{
    if (huart == nullptr)
        return false;

    is_waiting_for_header = true;
    if (HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2) != HAL_OK)
        return false;

    return true;
}

Bno08::BnoRecvStatus Bno08::rx_callback()
{
    BnoRecvStatus status;
    if (is_waiting_for_header)
    {
        if (buffer.header == BNO_HEADER)
        {
            is_waiting_for_header = false;
            HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer.index, 17);
            status = HEADER_MATCHED;
        }
        else
        {
            HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2);
            status = HEADER_ERROR;
        }
    }
    else
    {
        if (buffer.csum == calc_checksum())
        {
            parse_imu_data();
            last_receive = HAL_GetTick();
            status = CHECKSUM_MATCHED;
        }
        else
        {
            status = CHECKSUM_ERROR;
        }
        is_waiting_for_header = true;
        HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2);
    }
    return status;
}

inline void Bno08::parse_imu_data()
{
    data.yaw = -buffer.yaw * 0.01f;
    data.pitch = buffer.roll * 0.01f;
    data.roll = buffer.pitch * 0.01f;
    data.accel_x = buffer.accel_x * _g / 1000;
    data.accel_y = buffer.accel_y * _g / 1000;
    data.accel_z = buffer.accel_z * _g / 1000;
}

void Bno08::print_buffer()
{
    printf("bno_buf: %02x %hu %hd %hd %hd %hd %hd %hd %hu %hu %hu %02x\n",
           buffer.header,
           buffer.index,
           buffer.yaw,
           buffer.pitch,
           buffer.roll,
           buffer.accel_x,
           buffer.accel_y,
           buffer.accel_z,
           buffer.mi,
           buffer.mr,
           buffer.rsvd,
           buffer.csum);
}

inline uint8_t Bno08::calc_checksum()
{
    uint8_t csum = 0;
    uint8_t *pbyte = (uint8_t *)&buffer.index;
    for (int i = 0; i < 16; i++)
    {
        csum += *(pbyte++);
    }
    return csum;
}