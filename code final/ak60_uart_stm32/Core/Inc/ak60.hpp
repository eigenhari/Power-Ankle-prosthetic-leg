#include <usart.h>
#include <stdlib.h>
#include "buffer.h"
#include "ak60_uart_handle.hpp"

struct Parameters
{
    float mos_temp;
    float motor_temp;
    float output_current;
    float input_current;
    float id_current;
    float iq_current;
    float throttle;
    float speed;
    float input_voltage;
    int8_t reserved[24];
    float outer_loop_position;
    int8_t id;
    int16_t reserved_temperature[3];
    float vd_voltage;
    float vq_voltage;
};

typedef enum
{
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,             // Get motor operating parameters
    COMM_SET_DUTY,               // Motor operates in duty cycle mode
    COMM_SET_CURRENT,            // Motor operates in current loop mode
    COMM_SET_CURRENT_BRAKE,      // Motor operates in current brake mode
    COMM_SET_RPM,                // Motor operates in speed loop mode
    COMM_SET_POS,                // Motor operates in position loop mode
    COMM_SET_HANDBRAKE,          // Motor operates in handbrake current loop mode
    COMM_SET_DETECT,             // Motor real-time feedback current position command
    COMM_ROTOR_POSITION = 22,    // Motor feedback current position
    COMM_GET_VALUES_SETUP = 50,  // Motor single or multiple parameter acquisition command
    COMM_SET_POS_SPD = 91,       // Motor operates in position-speed loop mode
    COMM_SET_POS_MULTI = 92,     // Set motor motion to single-turn mode
    COMM_SET_POS_SINGLE = 93,    // Set motor motion to multi-turn mode, range ±100 turns
    COMM_SET_POS_UNLIMITED = 94, // Reserved
    COMM_SET_POS_ORIGIN = 95,    // Set motor origin
} DATA_FRAME;

typedef enum
{
    GET_POSITION
} OPERATION;

typedef enum
{
    BUSY,
    OK
} UART_STATE;

class AK60
{
public:
    DATA_FRAME mode;
    UART uart;
    OPERATION operation;
    uint8_t transmit_data_frame[TRANSMIT_BUFFER_SIZE];
    uint8_t receive_data_frame[RECEIVE_BUFFER_SIZE];
    Parameters parameters;

    AK60();
    AK60(UART_HandleTypeDef *);
    void init();
    void get_position();
    void set_position(float);
    void set_speed(float);
    void set_current(float);

    void set_position_speed(float, float, float);

    void send_command();
    void receive_data();
    void request_parameters();
    void update_parameters();
    void display_parameters();
    void set_home_position();
};