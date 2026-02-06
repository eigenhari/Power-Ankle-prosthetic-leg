#ifndef BNO08_HPP_
#define BNO08_HPP_

#include "usart.h"

#define BNO_PACKET_SIZE 19
#define BNO_HEADER 0xAAAA
#define BNO_TIME_PERIOD 10 // milliseconds for 100Hz
#define _g 9.80665f

#pragma pack(push, 1)
struct BnoData
{
    uint16_t header;
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint8_t mi;
    uint8_t mr;
    uint8_t rsvd;
    uint8_t csum;
};

#pragma pack(pop)

class Bno08
{
public:
    enum BnoRecvStatus
    {
        HEADER_MATCHED,
        HEADER_ERROR,
        CHECKSUM_MATCHED,
        CHECKSUM_ERROR
    };

    struct ImuData
    {
        float yaw;
        float pitch;
        float roll;
        float accel_x;
        float accel_y;
        float accel_z;
    };

    Bno08() : huart(nullptr) {}
    Bno08(UART_HandleTypeDef *_huart) : huart(_huart) {}
    Bno08 &operator=(const Bno08 &) = default;
    ~Bno08() = default;

    bool init();
    BnoRecvStatus rx_callback();
    void parse_imu_data();
    ImuData get_data();
    bool connected();
    void print_buffer();
    UART_HandleTypeDef *get_uart_handle();

private:
    UART_HandleTypeDef *huart;
    BnoData buffer;
    ImuData data;
    bool is_waiting_for_header;
    uint32_t last_receive = 0;

    uint8_t calc_checksum();
};

inline Bno08::ImuData Bno08::get_data()
{
    return data;
}

inline bool Bno08::connected()
{
    if (HAL_GetTick() - last_receive < 2 * BNO_TIME_PERIOD)
        return true;
    return false;
}

inline UART_HandleTypeDef *Bno08::get_uart_handle()
{
    return huart;
}

#endif // BNO08_HPP_