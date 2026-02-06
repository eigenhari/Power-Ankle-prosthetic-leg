/***********************************************************************************************
 **************************************************************************************/

#include <memory.h>
#include <stdio.h>

#include "ak60_uart_handle.hpp"

UART::UART() : huart(nullptr) {}

UART::UART(UART_HandleTypeDef *_huart) : huart(_huart) {}

bool UART::init()
{
    if (huart == nullptr)
        return false;

    receive_state = WAITING_FOR_FRAME_HEAD;
    // is_waiting_for_start_byte = true;
    is_transmitting = false;

    data_ready = false;

    HAL_UART_Receive_DMA(huart, &recv_frame_head, 1);

    // printf("uart initilized\n");

    return true;
}

/**
 * This function receives data over UART. It performs the following steps:
 * - Checks whether the start byte has been received.
 *   - If the start byte is received, proceeds to receive the data payload along with the CRC.
 *   - If the start byte is not received, continues waiting for it.
 * - Once the data is received, validates the CRC.
 * - Copies the data to the provided buffer and updates the reception status accordingly.
 *
 * @attention This function assumes that the UART has been initialized in RECEIVING mode or BOTH
 *            prior to calling this function. If the UART is initialized in a different mode,
 *            the behavior of this function may not be as expected.
 */
UARTReceiveStatus UART::process_receive_callback()
{
    /* Know the current tick */
    uint32_t current_tick = HAL_GetTick();
    // data_ready = false;
    /* Check whether received start byte or data */
    if (receive_state == WAITING_FOR_FRAME_HEAD)
    {
        // printf("frame_head :: %x\n", recv_frame_head);
        /* If it is start byte, check its value */
        if (recv_frame_head == FRAME_HEAD)
        {
            /* Reset start byte flag for receiving data */
            receive_state = WAITING_FOR_DATA_LENGTH;
            /* Trigger DMA fo2 31 04 01 41 00 F5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 r receiving data */
            HAL_UART_Receive_DMA(huart, recv_data_length, 2);

            /* Update received_status for start byte matched */
            received_status = UART_START_BYTE_MATCHED;

            // printf("frame head ok\n");
        }
        else /* Start byte failed */
        {
            /* Increase start byte error count for testing*/
            start_byte_error_count++;
            /* Call user defined error callback */
            UART_RxErrorCallBack(huart, UART_START_BYTE_ERROR);
            /* Again try to receive start byte */
            HAL_UART_Receive_DMA(huart, &recv_frame_head, 1);

            received_status = UART_START_BYTE_ERROR;

            // printf("start byte error: %0x\n", recv_frame_head);
        }
    }
    else if (receive_state == WAITING_FOR_DATA_LENGTH)
    {
        // printf("g : %x\n", recv_data_length[1]);
        if (recv_data_length[1] != 0x04)
        {
            HAL_UART_Receive_DMA(huart, &recv_frame_head, 1);
            receive_state = WAITING_FOR_FRAME_HEAD;
            return received_status;
        }

        HAL_UART_Receive_DMA(huart, receive_buffer, recv_data_length[0] + 2); // data(len) + checksum(2) + frame_tail(1)
        receive_state = WAITING_FOR_REMAINING_DATA;
    }
    else /* Receive data */
    {
        printf("data_length, data_frame :: %x, %x\n", recv_data_length[0], recv_data_length[1]);
        print_receive_buffer();
        if ((receive_buffer[recv_data_length[0] + 1]) != FRAME_TAIL)
        {
            // printf("Error in receiving data frame correctly occured!!! Frame Tail Error!!!\n");
            received_status = UART_FRAME_TAIL_ERROR;
            receive_state = WAITING_FOR_FRAME_HEAD;
            HAL_UART_Receive_DMA(huart, &recv_frame_head, 1);
            return received_status;
        }
        /* Calculate CRC for received data */
        uint16_t calculated_hash = get_crc16(&recv_data_length[1], recv_data_length[0]);
        uint16_t received_hash = ((uint16_t)receive_buffer[recv_data_length[0] - 1] << 8 |
                                  (uint16_t)receive_buffer[recv_data_length[0]]);
        if (calculated_hash == received_hash)
        {
            // printf("received ok\n");
            /* Put data from receive buffer to receive data*/
            // memcpy(receive_data, receive_buffer, );

            /* Update received_status for CRC Matched */
            received_status = UART_CRC_MATCHED;

            data_ready = true;

            receive_cplt_period = current_tick - last_receive_cplt_tick;
            last_receive_cplt_tick = current_tick;
        }
        else /* Hash didn't match */
        {
            // printf("Error occured in CRC Check\n");
            /* Increase check error count for testing*/
            check_error_count++;

            data_ready = false;

            /* Call user defined error callback */
            UART_RxErrorCallBack(huart, UART_CRC_ERROR);
            /* Update received_status for CRC failure */
            received_status = UART_CRC_ERROR;

            printf("crc error\n");
        }

        /* Set start byte flag to receive start byte again for the next packet */
        receive_state = WAITING_FOR_FRAME_HEAD;
        /* Trigger DMA for receiving start byte */
        HAL_UART_Receive_DMA(huart, &recv_frame_head, 1);
    }

    /* Update last_receive_tick to track receive call */
    last_receive_tick = current_tick;

    /* Return received_status */
    return received_status;
}

void UART::process_transmit_callback()
{
    /* Update transmit_cplt_period */
    uint32_t now = HAL_GetTick();
    transmit_cplt_period = now - last_transmit_cplt_tick;
    last_transmit_cplt_tick = now;
    is_transmitting = false;
}

/**
 * This function receives data over UART. It checks whether the data has been received
 * and copies the data to the provided buffer.
 */
bool UART::get_received_data(uint8_t *r_data)
{
    // if (receive_seq != prev_receive_seq)
    if (data_ready)
    {
        /* Copy the data*/
        memcpy(r_data, receive_buffer, recv_data_length[0]);
        /*Update prev_receive_seq so we can use it in next receive*/
        // prev_receive_seq = receive_seq;

        data_ready = false;

        return true;
    }

    return false;
}

/**
 * This function transmits data over UART. It prepares the data packet by loading
 * the start byte, copying the transmitted data, calculating and loading the CRC,
 * then transmitting the data using DMA. It also updates the transmit tick.
 *
 * @note This function assumes that the UART communication has been initialized
 *       prior to calling this function.
 */
bool UART::transmit(uint8_t *t_data, uint8_t len)
{
    /* Load start byte */
    transmit_data_length = len;
    int i = 0;
    transmit_buffer[i++] = FRAME_HEAD;
    transmit_buffer[i++] = len;

    /* Load tx data */
    memcpy(transmit_buffer + i, t_data, len);
    i = i + len;
    uint16_t crc = get_crc16(t_data, len);
    transmit_buffer[i++] = crc >> 8;
    transmit_buffer[i++] = crc;
    // memcpy(transmit_buffer + i, (uint8_t *)&crc, 2);

    transmit_buffer[i++] = FRAME_TAIL;

    // print_transmit_buffer();
    /* Transmit tx data using DMA */

    if (HAL_UART_Transmit_DMA(huart, transmit_buffer, i) == HAL_OK)
    {
        /* Update transmit tick */
        last_transmit_tick = HAL_GetTick();
        /* Update is_transmitting flag */
        is_transmitting = true;
        return true;
    }

    /* Increase transmit error count for testing*/
    transmit_error_count++;
    /* Call user defined error callback */
    UART_TxErrorCallBack(huart);

    return false;
}

USART_TypeDef *UART::get_uart_instance()
{
    return huart->Instance;
}

bool UART::connected()
{
    return (HAL_GetTick() - last_receive_cplt_tick < UART_CONNCETION_TIMEOUT);
}

uint32_t UART::get_last_receive_tick()
{
    return last_receive_tick;
}

uint32_t UART::get_last_transmit_tick()
{
    return last_transmit_tick;
}

uint32_t UART::get_last_receive_cplt_tick()
{
    return last_receive_cplt_tick;
}

uint32_t UART::get_receive_cplt_period()
{
    return receive_cplt_period;
}

float UART::get_receive_cplt_frequency()
{
    if (receive_cplt_period == 0)
        return 0.0f;

    return 1000.0f / receive_cplt_period;
}

uint32_t UART::get_receive_throughput(uint32_t time)
{
    if (receive_cplt_period == 0 || time == 0)
        return 0;

    return (recv_data_length[0] + 2) * time / receive_cplt_period;
}

// uint32_t UART::get_receive_seq()
// {
//     return receive_seq;
// }

uint32_t UART::get_transmit_cplt_period()
{
    return transmit_cplt_period;
}

float UART::get_transmit_cplt_frequency()
{
    if (transmit_cplt_period == 0)
        return 0.0f;

    return 1000.0f / transmit_cplt_period;
}

uint32_t UART::get_transmit_throughput(uint32_t time)
{
    if (transmit_cplt_period == 0 || time == 0)
        return 0;

    return (transmit_data_length + 2) * time / transmit_cplt_period;
}

void UART::print_uart_status()
{
    printf("========================================\n");
    printf("UART Status\n");
    printf("========================================\n");
    printf("Receive Status:\n");
    printf("----------------------------------------\n");
    // printf("is_waiting_for_start_byte:: %d\n", is_waiting_for_start_byte);
    // printf("Receive Seq:: %hu\n", receive_seq);
    printf("Start Byte Error Count:: %lu\n", start_byte_error_count);
    printf("Check Error Count:: %lu\n", check_error_count);
    printf("Last Receive Tick:: %lu\n", last_receive_tick);
    printf("Last Transmit Tick:: %lu\n", last_transmit_tick);
    printf("Last Receive Cplt Tick:: %lu\n", last_receive_cplt_tick);
    printf("Last Transmit Cplt Tick:: %lu\n", last_transmit_cplt_tick);
    printf("Receive Cplt Period:: %lu\n", receive_cplt_period);
    printf("Receive Cplt Frequency:: %.2f\n", get_receive_cplt_frequency());
    printf("Receive Throughput:: %lu\n", get_receive_throughput());
    printf("----------------------------------------\n");
    printf("Transmit Status:\n");
    printf("----------------------------------------\n");
    printf("is_transmitting:: %d\n", is_transmitting);
    printf("Transmit Cplt Period:: %lu\n", transmit_cplt_period);
    printf("Transmit Cplt Frequency:: %.2f\n", get_transmit_cplt_frequency());
    printf("Transmit Throughput:: %lu\n", get_transmit_throughput());
    printf("========================================\n\n");
}

void UART::print_receive_buffer()
{
    printf("receive buffer\n");
    for (int i = 0; i < recv_data_length[0] + 2; i++)
    {
        printf(" %02x", receive_buffer[i]);
    }
    printf("\n");
}

void UART::print_transmit_buffer()
{
    printf("UART TX Data::");
    for (int i = 0; i < transmit_data_length + 5; ++i)
    {
        printf("%02x ", transmit_buffer[i]);
    }
    printf("\n");
}

/*Weakly defined, so user can have its own implementation and donot throw error if not defined*/
__weak void UART::UART_RxErrorCallBack(UART_HandleTypeDef *huart, UARTReceiveStatus status)
{
    (void)huart;
    (void)status;
}

/*Weakly defined, so user can have its own implementation and donot throw error if not defined*/
__weak void UART::UART_TxErrorCallBack(UART_HandleTypeDef *huart)
{
    (void)huart;
}

UARTReceiveStatus UART::get_receive_status()
{
    return received_status;
}