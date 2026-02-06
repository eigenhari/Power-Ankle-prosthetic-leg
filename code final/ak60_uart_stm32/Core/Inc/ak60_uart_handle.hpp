/**
 ******************************************************************************

 ******************************************************************************
 */

#ifndef UART_HPP__
#define UART_HPP__

#include "stm32f4xx_hal.h"
#include "crc16.hpp"
#include <stdlib.h>

#define TRANSMIT_BUFFER_SIZE 32
#define RECEIVE_BUFFER_SIZE 256
#define FRAME_HEAD 0x02
#define FRAME_TAIL 0x03

#ifndef UART_CONNCETION_TIMEOUT
#define UART_CONNCETION_TIMEOUT 100 /**< Timeout for UART connection */
#endif

enum UARTMode
{
    UART_RECEIVING,    /**< UART in receiving mode */
    UART_TRANSMITTING, /**< UART in transmitting mode */
    UART_BOTH          /**< UART in both receiving and transmitting mode */
};

/**
 * @brief Enumeration defining the status of UART reception
 */
enum UARTReceiveStatus
{
    UART_START_BYTE_MATCHED, /**< Start byte of packet matched */
    UART_START_BYTE_ERROR,   /**< Start byte of packet failed to match */
    UART_CRC_MATCHED,        /**< CRC hash matched */
    UART_CRC_ERROR,          /**< CRC hash failed to match */
    UART_INTERRUPT_CRASHED,   /**< UART Interrupt crashed while receiving*/
    UART_FRAME_TAIL_ERROR
};

enum UART_Receive_state{
    WAITING_FOR_FRAME_HEAD,
    WAITING_FOR_DATA_LENGTH,
    WAITING_FOR_REMAINING_DATA
};
/**
 * @brief Class representing the UART communication functionality
 */
class UART
{
public:
    /**
     * @brief Default constructor for UART class
     * @warning Be careful, do not call init() if uart handle is not set
     */
    UART();

    /**
     * @brief Constructor for UART class
     * @param[in] _huart Pointer to the UART handle
     * @param[in] _mode UART Mode
     * @param[in] _rt_size Size of receiving or transmitting data
     * @param[in] _t_size Size of transmittin data used if r_size and t_size are different
     */
    UART(UART_HandleTypeDef *_huart);

    /**
     * @brief Default copy constructor for UART class
     */
    UART(const UART &) = default;

    /**
     * @brief Default assignment operator for UART class
     */
    UART &operator=(const UART &) = default;

    /**
     * @brief Default destructor for UART class
     */
    ~UART() = default;

    /**
     * @brief Initialize UART communication
     */
    bool init();

    /**
     * @brief Parse the received data and check for start byte, data, and CRC
     */
    UARTReceiveStatus process_receive_callback();

    /**
     * @brief Process transmit callback to update status
     */
    void process_transmit_callback();

    /**
     * @brief Copy data from the received data buffer to the provided buffer
     * @param[in] Pointer to the data to be received
     */
    bool get_received_data(uint8_t *data);

    /**
     * @brief Transmit data over UART in blocking mode
     * @param[in] Pointer to the data to be transmitted
     * @return True if the data is transmitted successfully, false otherwise
     */
    bool transmit(uint8_t *, uint8_t);

    /**
     * @brief Get the UART instance
     * @return Pointer to the UART handle
     */
    USART_TypeDef *get_uart_instance();

    /**
     * @brief Check if the UART is connected
     * @return True if connected, false otherwise
     */
    bool connected();

    /**
     * @brief Get the tick count of the last received data
     * @return Tick count of the last received data
     */
    uint32_t get_last_receive_tick();

    /**
     * @brief Get the tick count of the last data reception completion
     * @return Tick count of the last data reception completion
     */
    uint32_t get_last_receive_cplt_tick();

    /**
     * @brief Get the tick count of the last data transmission
     * @return Tick count of the last data transmission
     */
    uint32_t get_last_transmit_tick();

    /**
     * @brief Get the tick count of the last data transmission completion
     * @return Tick count of the last data transmission completion
     */
    uint32_t get_last_transmit_cplt_tick();

    /**
     * @brief Get the period of last data reception completion
     * @return Period of last data reception completion
     */
    uint32_t get_receive_cplt_period();

    /**
     * @brief Get the frequency of last data reception completion
     * @return Frequency(Hz) of last data reception completion
     */
    float get_receive_cplt_frequency();

    /**
     * @breie Get Receive Throughput
     * @param[in] time Time(ms) for bytes count [defualt 1000]
     * @return Receive Throughput in bytes per time
     */
    uint32_t get_receive_throughput(uint32_t time = 1000);

    /**
     * @brief Get the sequence number of the received data
     * @return Sequence number of the received data
     */
    // uint32_t get_receive_seq();

    /**
     * @brief Get the period of last data transmit completion
     * @return Period of last data transmit completion
     */
    uint32_t get_transmit_cplt_period();

    /**
     * @brief Get the frequency of last data transmit completion
     * @return Frequency(Hz) of last data transmit completion
     */
    float get_transmit_cplt_frequency();

    /**
     * @breie Get Transmit Throughput
     * @param[in] time Time(ms) for bytes count [defualt 1000]
     * @return Transmit Throughput in bytes per time
     */
    uint32_t get_transmit_throughput(uint32_t time = 1000);

    /**
     * @brief Print UART status
     */
    void print_uart_status();

    /**
     * @brief Display the latest received packet in hexadecimal format
     */
    void print_receive_buffer();

    /**
     * @brief Display the transmitting data in hexadecimal format
     */
    void print_transmit_buffer();

    /**
     * @brief  Recieve Error handler function
     */
    static void UART_RxErrorCallBack(UART_HandleTypeDef *huart, UARTReceiveStatus status);

    /**
     * @brief Transmit Error handler function
     */
    static void UART_TxErrorCallBack(UART_HandleTypeDef *huart);


    UARTReceiveStatus get_receive_status();
    bool data_ready = false;

private:
    UART_HandleTypeDef *huart; /**< Pointer to the UART handle */
    UART_Receive_state receive_state;
    // uint8_t data_length;

    uint8_t recv_frame_head, recv_data_length[2], receive_buffer[RECEIVE_BUFFER_SIZE];   /**< Array to receive data */
    uint8_t transmit_buffer[TRANSMIT_BUFFER_SIZE];  /**< Array to transmit start byte, data, hash */
    // uint8_t receive_data[UART_MAX_RX_BUFFER_SIZE - 2]; /**< Array to receive data */
    // uint16_t r_size;                                   /**< Sizes for receiving data */
    uint8_t transmit_data_length;                                   /**< Sizes for tarnsmitting data */

    uint32_t last_receive_tick = 0;       /**< last receive function call tick  */
    uint32_t last_transmit_tick = 0;      /**< last  transmit function call tick */
    uint32_t last_receive_cplt_tick = 0;  /**< last receive complete tick after crc matched */
    uint32_t last_transmit_cplt_tick = 0; /**< last transmit complete tick after transmit cplt callback */
    uint32_t receive_cplt_period = 0;     /**< Period of last receive complete */
    uint32_t transmit_cplt_period = 0;    /**< Period of last transmit complete */

    // bool is_waiting_for_start_byte = true;      /**< Current state of receiving */
    // bool is_waiting_for_data_length = false;
    bool is_transmitting;                /**< Current state of transmitting */
    // UARTMode mode;                       /**< Current UART mode */
    UARTReceiveStatus received_status;   /**< last receive status */
    // uint16_t receive_seq = 0;            /**< Sequence number of received data */
    // uint16_t prev_receive_seq = 0;       /**< Previous sequence number of received data */
    uint32_t start_byte_error_count = 0; /**< Count of start byte errors */
    uint32_t check_error_count = 0;      /**< Count of CRC hash errors */
    uint32_t transmit_error_count = 0;   /**< Count of transmit errors */
};

#endif // UART_HPP__