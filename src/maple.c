/**
 * maple.c
 *
 * Handle reading and writing to Sega Dreamcast Maple Bus using PIO
 *
 * Copyright (c) 2024 Colin Luoma
 */

#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <hardware/pio.h>
#include <hardware/gpio.h>

#include "maple_rx.pio.h"
#include "maple_tx.pio.h"
#include "maple.h"
#include "dw_vmu_static_data.h"
#include "utils.h"

// note that PIO pins are sequential, so MAPLE PIN 5 will be GPIO 17
#define PIO_MAPLE_PIN_1 16
//#define MAPLE_TX_CLOCK_DIVIDER 9.375f   // should equal 62.5ns when sysclock is 150MHz
#define MAPLE_TX_CLOCK_DIVIDER 8.904f
#define MAPLE_RX_CLOCK_DIVIDER 1.0f     // for reading we look for changes so want as fast as possible

#define MAPLE_WAIT_FOR_RESPONSE_TIMEOUT_US 5000

#define FRAME_PAYLOAD_WORD(X, Y) ( (uint32_t)((X->payload)[Y]) | (((uint32_t)(X->payload)[Y+1]) << 8) | (((uint32_t)(X->payload)[Y+2]) << 16) | (((uint32_t)(X->payload)[Y+3]) << 24) )

typedef enum {
    MP_OK = 0,
    MP_TIMEOUT,
    MP_ERROR,
    MP_CRC_MISMATCH = -1
} mp_error_codes_t;

typedef enum {
    CONTROLLER_PORT_1 = 0,
    CONTROLLER_PORT_2,
    CONTROLLER_PORT_3,
    CONTROLLER_PORT_4
} mp_controller_port_t;

typedef enum {
    DREAMCAST_ADDR   = 0x00,
    MAIN_PERIPHERAL  = 0x20,
    SUB_PERIPHERAL_5 = 0x10,
    SUB_PERIPHERAL_4 = 0x08,
    SUB_PERIPHERAL_3 = 0x04,
    SUB_PERIPHERAL_2 = 0x02,
    SUB_PERIPHERAL_1 = 0x01
} mp_peripheral_addr_t;

enum mp_command_code {
    REQUEST_DEVICE_INFO             = 1,
    REQUEST_EXTENDED_DEVICE_INFO    = 2,
    RESET_DEVICE                    = 3,
    SHUTDOWN_DEVICE                 = 4,
    DEVICE_INFO_RESPONSE            = 5,
    EXTENDED_DEVICE_INFO_RESPONSE   = 6,
    COMMAND_ACK_RESPONSE            = 7,
    DATA_TRANSFER_RESPONSE          = 8,
    GET_CONDITION                   = 9,
    GET_MEMORY_INFORMATION          = 10,
    BLOCK_READ                      = 11,
    BLOCK_WRITE                     = 12,
    GET_LAST_ERROR                  = 13,
    SET_CONDITION                   = 14,
    NO_RESPONSE                     = (uint8_t)-1,
    FUNCTION_CODE_UNSUPPORTED_RESPONSE = (uint8_t)-2,
    UNKNOWN_COMMAND_RESPONSE        = (uint8_t)-3,
    PLEASE_SEND_AGAIN_RESPONSE      = (uint8_t)-4,
    FILE_ERROR_RESPONSE             = (uint8_t)-5
};

// used to map data received in the rx_buffer
// don't ever create one of these structs, just cast the rx_buffer to one
struct __attribute__((packed)) mp_rx_frame {
    uint8_t word_count;
    uint8_t sender_addr;
    uint8_t recipient_addr;
    uint8_t command;
    uint8_t payload[RX_BUFFER_SIZE - 4];
};

struct __attribute__((packed)) mp_tx_frame {
    uint8_t word_count;             // number of 4-byte words in payload
    uint8_t sender_addr;            // address of sending device
    uint8_t recipient_addr;         // address of receiving device (usually the console)
    uint8_t command;                // command code
    uint8_t *payload;               // pointer to additional packet data
};

struct __attribute__((packed)) condition_controller {
    uint32_t func;                  // function code (big endian)
    uint16_t buttons;               // buttons bitfield (little endian)
    uint8_t triggerr;               // right analogue trigger (0-255)
    uint8_t triggerl;               // left analogue trigger (0-255)
    uint8_t joyx;                   // analogue joystick X (0-255)
    uint8_t joyy;                   // analogue joystick Y (0-255)
    uint8_t joyx2;                  // second analogue joystick X (0-255)
    uint8_t joyy2;                  // second analogue joystick Y (0-255)
};

// Main peripheral condition
struct condition_controller cc_frame_payload;
controller_condition_t cond = DEFAULT_CONTROLLER_CONDITION;

/**
 * Populates a mp_device_info from Maple GET_INFORMATION response data.
 */
void parse_device_info_frame(struct mp_rx_frame * rx_frame, mp_device_info_t * di)
{
    // Fix the byte order of the payload
    uint32_t *payload = (uint32_t *)rx_frame->payload;
    for (int i = 0; i < rx_frame->word_count; i++)
    {
        payload[i] = __builtin_bswap32(payload[i]);
    }

    memcpy(di, (rx_frame)->payload, sizeof(mp_device_info_t));

    // Fix byte order of certain entries
    di->func = __builtin_bswap32(di->func);
    di->function_data[0] = __builtin_bswap32(di->function_data[0]);
    di->function_data[1] = __builtin_bswap32(di->function_data[1]);
    di->function_data[2] = __builtin_bswap32(di->function_data[2]);

    if (rx_frame->sender_addr & MAIN_PERIPHERAL && di->func & FUNC_CONTROLLER)
    {
        DEBUG_PRINT("GOT CONTROLLER\r\n");
        DEBUG_PRINT("\"%.30s\"\r\n", di->product_name);
    }
}

/**
 * Populates cc_frame_payload from Maple response data.
 */
void parse_data_transfer_response(struct mp_rx_frame * rx_frame)
{
    // Fix the byte order of the payload
    uint32_t *payload = (uint32_t *)rx_frame->payload;
    for (int i = 0; i < rx_frame->word_count; i++)
    {
        payload[i] = __builtin_bswap32(payload[i]);
    }

    cc_frame_payload = *(struct condition_controller *)(rx_frame)->payload;
    // Fix byte order of certain entries
    cc_frame_payload.func = __builtin_bswap32(cc_frame_payload.func);

    if ( !(cc_frame_payload.func & FUNC_CONTROLLER) )
        return;

    cond.a = ~cc_frame_payload.buttons & (1 <<  2) ? 1 : 0;
    cond.b = ~cc_frame_payload.buttons & (1 <<  1) ? 1 : 0;
    cond.x = ~cc_frame_payload.buttons & (1 << 10) ? 1 : 0;
    cond.y = ~cc_frame_payload.buttons & (1 <<  9) ? 1 : 0;
    cond.d_up = ~cc_frame_payload.buttons & (1 <<  4) ? 1 : 0;
    cond.d_down = ~cc_frame_payload.buttons & (1 <<  5) ? 1 : 0;
    cond.d_left = ~cc_frame_payload.buttons & (1 <<  6) ? 1 : 0;
    cond.d_right = ~cc_frame_payload.buttons & (1 <<  7) ? 1 : 0;
    cond.start = ~cc_frame_payload.buttons & (1 <<  3) ? 1 : 0;

    cond.trigger_left = cc_frame_payload.triggerl;
    cond.trigger_right = cc_frame_payload.triggerr;
    cond.joy_x = cc_frame_payload.joyx;
    cond.joy_y = cc_frame_payload.joyy;
}

/**
 * Attempts to get a word from the RX FIFO, waiting for up to timeout_us microseconds.
 * timeout_us is updated to the remaining time of the timeout. -1 is returned if the timeout was reached.
 * If timeout is reached, return value is invalid.
 */
static inline uint32_t pio_sm_get_timeout(PIO pio, uint sm, int64_t * timeout_us)
{
    absolute_time_t end_time = make_timeout_time_us(*timeout_us);
    while (pio_sm_is_rx_fifo_empty(pio, sm))
    {
        *timeout_us = absolute_time_diff_us(get_absolute_time(), end_time);
        if (*timeout_us < 0)
        {
            *timeout_us = -1;
            return 0;
        }
    }
    return pio_sm_get(pio, sm);
}

/**
 * Reads a MAPLE data packet from PIO. Try until we get a full packet, or we timeout.
 */
int read_maple(mp_dreamcast_t * mpb)
{
    PIO pio     = mpb->rx_pio.pio;
    uint sm     = mpb->rx_pio.sm;
    uint offset = mpb->rx_pio.offset;

    enum read_state {IDLE=0, READING};

    enum read_state state = IDLE;
    uint8_t checksum = 0;
    uint32_t upper = 0b11111100;
    uint32_t lower = 0b00000011;
    uint8_t cur_byte = 0;
    uint8_t prev_byte = 0;
    uint8_t packet_byte;
    uint32_t bytes_read = 0;
    uint32_t max_bytes = 5;  // 4 header bytes plus the CRC checksum

    // Clear up some internal SM state
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));

    // Bring Maple pin 2 high before reading
    gpio_pull_up(PIO_MAPLE_PIN_1+1);
    gpio_set_dir(PIO_MAPLE_PIN_1+1, GPIO_IN);

    while (true)
    {
        int64_t timeout = MAPLE_WAIT_FOR_RESPONSE_TIMEOUT_US;
        uint32_t word = pio_sm_get_timeout(pio, sm, &timeout);
        if (timeout < 0)
            return MP_TIMEOUT;

        cur_byte = (word & 0x000000FF);
        if (state == IDLE)
        {
            if ( (cur_byte & upper) == 0b10000100)
            {  // check for start of packet
                state = READING;
                max_bytes = 5;
                bytes_read = 0;
            }
            else
            {  // read error, bad timing, or got some garbage data, try again
                DEBUG_PRINT("read error\r\n");
                printf("read error %08b\r\n", cur_byte);
                maple_rx_sm_reset(pio, sm);
                continue;
            }
        }
        else
        {
            packet_byte = ((prev_byte & lower) << 6) | ((cur_byte & upper) >> 2);
            //printf("%08b\n", packet_byte);
            // add byte to buffer
            mpb->rx_buffer[bytes_read] = packet_byte;
            bytes_read += 1;
            if (bytes_read == 1)
            {
                max_bytes += packet_byte*4;
            }
            else if (bytes_read == max_bytes)
            {
                if (checksum != packet_byte)
                    return MP_CRC_MISMATCH;
                return MP_OK;
            }
            checksum ^= packet_byte;
        }
        prev_byte = cur_byte;
    }
}

/**
 * Writes a MAPLE data packet to PIO
 */
void write_maple(mp_dreamcast_t * mpb, struct mp_tx_frame * frame)
{
    PIO pio = mpb->tx_pio.pio;
    uint sm = mpb->tx_pio.sm;

    // set pin direction to OUT, pins HIGH, and start state machine
    pio_gpio_init(pio, PIO_MAPLE_PIN_1+1);
    pio_sm_set_pins(pio, sm, (1u << PIO_MAPLE_PIN_1) | (1u << (PIO_MAPLE_PIN_1+1)) );
    pio_sm_set_pindirs_with_mask(pio, sm, (1u << PIO_MAPLE_PIN_1) | (1u << (PIO_MAPLE_PIN_1+1)), (1u << PIO_MAPLE_PIN_1) | (1u << (PIO_MAPLE_PIN_1+1)));
    pio_sm_set_enabled(pio, sm, true);

    uint32_t checksum = 0;
    uint32_t tx_buf_ind = 0;

    // Frame header
    for (uint32_t i = 0; i < 4; i++)
    {
        mpb->tx_buffer[tx_buf_ind++] = ((uint8_t*) frame)[i];
        checksum ^= ((uint8_t*) frame)[i];
    }
    // Payload
    for (uint32_t i = 3; i < frame->word_count*4; i+=4)
    {
        for (uint32_t j = 0; j < 4; j++)
        {
            uint8_t tmp = frame->payload[i-j];
            mpb->tx_buffer[tx_buf_ind++] = tmp;
            checksum ^= tmp;
        }
    }

    // CRC checksum
    mpb->tx_buffer[tx_buf_ind++] = checksum;

    // Send data to PIO state machine
    for (uint32_t i = 0; i < tx_buf_ind; i++)
    {
        pio_sm_put_blocking(pio, sm, mpb->tx_buffer[i] << 24);
    }

    // wait for PIO to clear its fifo
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) tight_loop_contents();

    // wait for PIO to stall waiting for more data
    // clear any previous sticky stall status.
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + mpb->tx_pio.sm);
    // wait until the stall flag is up again.
    while (!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)))) tight_loop_contents();

    // keep the line low until read is initiated
    gpio_set_dir(PIO_MAPLE_PIN_1+1, GPIO_OUT);
    gpio_put(PIO_MAPLE_PIN_1+1, 0);
    gpio_set_function(PIO_MAPLE_PIN_1+1, GPIO_FUNC_SIO);
    gpio_pull_down(PIO_MAPLE_PIN_1+1);

    // disable state machine
    pio_sm_set_consecutive_pindirs(pio, sm, PIO_MAPLE_PIN_1, 2, false);
    pio_sm_set_enabled(pio, sm, false);
}

/**
 * Populate a Maple packet with the given addresses, command code, and payload before sending to PIO
 */
void send_frame(mp_dreamcast_t * mpb, uint8_t sender_addr, uint8_t recipient_addr, uint8_t command_code, uint8_t *payload, uint8_t payload_len)
{
    struct mp_tx_frame f = {
            .word_count = payload_len,
            .sender_addr = sender_addr,
            .recipient_addr = recipient_addr,
            .command = command_code,
            .payload = (uint8_t *)payload};
    write_maple(mpb, &f);
}

mp_error_codes_t mp_request_and_response(mp_dreamcast_t * mpb, uint8_t sender_addr, uint8_t recipient_addr, uint8_t command_code, uint8_t *payload, uint8_t payload_len)
{
    send_frame(mpb, sender_addr, recipient_addr, command_code, payload, payload_len);

    int r = read_maple(mpb);

    if (r == MP_CRC_MISMATCH)
    {
        DEBUG_PRINT("MAPLE READ CRC MISMATCH\r\n");
        return r;
    }
    if (r == MP_TIMEOUT)
    {
        DEBUG_PRINT("MAPLE_READ TIMEOUT\r\n");
        return r;
    }

    return MP_OK;
}

void print_frame(struct mp_rx_frame *f)
{
#ifdef DEBUG
    printf("Printing Frame:\n");
    printf("Word count:\t%d\n", (int)f->word_count);
    printf("Sender:\t\t%08b\n", f->sender_addr);
    printf("Recipient:\t%08b\n", f->recipient_addr);
    printf("Command:\t%08b\n", f->command);

    printf("Payload [");
    for (int i = 0; i < (int)f->word_count; i++) {
        printf("%08x", ((uint32_t *)f->payload)[i]);
        if (i != (int)f->word_count - 1)
            printf(" ");
    }
    printf("]\r\n");
#endif
}


void rx_pio_init(mp_pio_block_t * pb)
{
    pb->pio = pio0;
    pb->sm = 0;
    pb->offset = pio_add_program(pb->pio, &maple_rx_program);
    maple_rx_program_init(pb->pio, pb->sm, pb->offset, PIO_MAPLE_PIN_1, MAPLE_RX_CLOCK_DIVIDER);
}

void tx_pio_init(mp_pio_block_t * pb)
{
    pb->pio = pio1;
    pb->sm = 0;
    pb->offset = pio_add_program(pb->pio, &maple_tx_program);
    maple_tx_program_init(pb->pio, pb->sm, pb->offset, PIO_MAPLE_PIN_1, MAPLE_TX_CLOCK_DIVIDER);
    pio_sm_set_enabled(pb->pio, pb->sm, false);
}

void mp_dreamcast_init(mp_dreamcast_t * mpb)
{
    mpb->state = MP_DREAMCAST_STATE_INITIALIZING;

    /// Some initial setup
    gpio_init(PIO_MAPLE_PIN_1+1);
    gpio_pull_up(PIO_MAPLE_PIN_1);
    gpio_pull_up(PIO_MAPLE_PIN_1+1);

    // Setup PIO state machines
    rx_pio_init(&mpb->rx_pio);
    tx_pio_init(&mpb->tx_pio);

    // Clear buffers
    memset(mpb->rx_buffer, 0, RX_BUFFER_SIZE);
    memset(mpb->tx_buffer, 0, TX_BUFFER_SIZE);
    pio_sm_clear_fifos(mpb->rx_pio.pio, mpb->rx_pio.sm);
    pio_sm_clear_fifos(mpb->tx_pio.pio, mpb->tx_pio.sm);

    // This is our sender address (it impersonates a Dreamcast)
    mpb->address = CONTROLLER_PORT_1 << 6 | DREAMCAST_ADDR;
    // This will get filled in by any conected peripherals
    mpb->peripheral_address = 0;

    // Clear any connected peripherals
    for (int i = 0; i < MP_MAX_CONNECTED_PERIPHERALS; i++)
    {
        mp_peripheral_init(&mpb->peripherals[i]);
    }

    mpb->state = MP_DREAMCAST_STATE_READY;
}

/**
 * Request a Device Information response from a target peripheral address
 */
mp_error_codes_t request_device_information(mp_dreamcast_t * mpb, mp_peripheral_addr_t target_addr)
{
    struct mp_rx_frame * rx_frame = (struct mp_rx_frame *)mpb->rx_buffer;

    send_frame(mpb, DREAMCAST_ADDR, target_addr, REQUEST_DEVICE_INFO, NULL, 0);
    int r = read_maple(mpb);

    if (r == MP_CRC_MISMATCH)
    {
        DEBUG_PRINT("MAPLE READ CRC MISMATCH\r\n");
        return r;
    }
    if (r == MP_TIMEOUT)
    {
        DEBUG_PRINT("MAPLE_READ TIMEOUT\r\n");
        return r;
    }

    if (rx_frame->command != DEVICE_INFO_RESPONSE)
    {
        DEBUG_PRINT("MAPLE_READ UNEXPECTED RESPONSE TYPE\r\n");
        return MP_ERROR;
    }

    DEBUG_PRINT("DEVICE_INFO_RESPONSE\r\n");
    switch(target_addr)
    {
        case MAIN_PERIPHERAL:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[0],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        case SUB_PERIPHERAL_1:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[1],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        case SUB_PERIPHERAL_2:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[2],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        default:
            break;
    }
    return MP_OK;
}

/**
 * Request a Device Information response from a target peripheral address
 */
mp_error_codes_t request_get_condition(mp_dreamcast_t * mpb, mp_peripheral_addr_t target_addr)
{
    struct mp_rx_frame * rx_frame = (struct mp_rx_frame *)mpb->rx_buffer;

    int r = mp_request_and_response(mpb, DREAMCAST_ADDR, target_addr, GET_CONDITION, NULL, 0);
    if ( r != MP_OK)
        return r;

    if (rx_frame->command != DATA_TRANSFER_RESPONSE)
    {
        DEBUG_PRINT("MAPLE_READ UNEXPECTED RESPONSE TYPE\r\n");
        return MP_ERROR;
    }

    DEBUG_PRINT("DATA_TRANSFER_RESPONSE\r\n");
    switch(target_addr)
    {
        case MAIN_PERIPHERAL:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[0],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        case SUB_PERIPHERAL_1:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[1],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        case SUB_PERIPHERAL_2:
            mp_peripheral_parse_req_device_info_resp(&mpb->peripherals[2],
                rx_frame->sender_addr, rx_frame->payload, rx_frame->word_count);
            break;
        default:
            break;
    }
    return MP_OK;
}

void mp_push_vmu_screen_data(mp_dreamcast_t * mpb, uint8_t * data, uint32_t len)
{
    for (int i = 0; i < 3; i++)
    {
        if (mpb->peripherals[i].type == MP_PERIPHERAL_VMU)
        {
            //printf("adding screen data\r\n");
            memcpy(mpb->peripherals[i].data.vmu.vmu_screen_buffer+8, data, len);
            mpb->peripherals[i].data.vmu.vmu_refresh = true;
        }
    }
}

/**
 * Run the main Maple loop from the Dreamcast's pov.
 * It is a state machine that waits until it gets a device information response from a connected Main Peripheral before
 * starting to request the peripherals condition.
 * A timeout while waiting for a response will trigger returning to looking for a Maine Peripheral.
 */
void mp_dreamcast_run(mp_dreamcast_t * mpb)
{
    if (mpb->state != MP_DREAMCAST_STATE_READY)
        return;

    // First we must query for a main peripheral
    mpb->state = MP_DREAMCAST_STATE_SEARCHING_FOR_MAIN_PERIPHERAL;


    while (1) {

        // Make sure we have a main peripheral connected
        if (mpb->state == MP_DREAMCAST_STATE_SEARCHING_FOR_MAIN_PERIPHERAL)
        {
            while (request_device_information(mpb, MAIN_PERIPHERAL) != MP_OK)
            {
                tight_loop_contents();
            }
            mpb->state = MP_DREAMCAST_STATE_RUNNING;
            mpb->peripheral_address |= MAIN_PERIPHERAL;
        }

        // Check for peripherals removed or inserted
        if ( (mpb->peripherals[0].address & SUB_PERIPHERAL_1) && !(mpb->peripheral_address & SUB_PERIPHERAL_1) )
        {
            // Sub 1 inserted
            DEBUG_PRINT("Sub-peripheral 1 inserted\r\n");
            if (request_device_information(mpb, SUB_PERIPHERAL_1) == MP_OK)
            {
                mpb->peripheral_address |= SUB_PERIPHERAL_1;
            }
        }
        else if ( !(mpb->peripherals[0].address & SUB_PERIPHERAL_1) && (mpb->peripheral_address & SUB_PERIPHERAL_1) )
        {
            // Sub 1 removed
            DEBUG_PRINT("Sub-peripheral 1 removed\r\n");
            mp_peripheral_init(&mpb->peripherals[1]);
            mpb->peripheral_address &= ~((uint8_t)SUB_PERIPHERAL_1);
        }
        if ( (mpb->peripherals[0].address & SUB_PERIPHERAL_2) && !(mpb->peripheral_address & SUB_PERIPHERAL_2) )
        {
            // Sub 2 inserted
            DEBUG_PRINT("Sub-peripheral 2 inserted\r\n");
            if (request_device_information(mpb, SUB_PERIPHERAL_2) == MP_OK)
            {
                mpb->peripheral_address |= SUB_PERIPHERAL_2;
            }
        }
        else if ( !(mpb->peripherals[0].address & SUB_PERIPHERAL_2) && (mpb->peripheral_address & SUB_PERIPHERAL_2) )
        {
            // Sub 2 removed
            DEBUG_PRINT("Sub-peripheral 2 removed\r\n");
            mp_peripheral_init(&mpb->peripherals[2]);
            mpb->peripheral_address &= ~((uint8_t)SUB_PERIPHERAL_2);
        }

        // Update VMU display
        if (mpb->peripheral_address & SUB_PERIPHERAL_1 &&
            mpb->peripherals[1].type == MP_PERIPHERAL_VMU &&
            mpb->peripherals[1].data.vmu.vmu_refresh == true)
        {
            //printf("sending screen data to vmu\r\n");
            send_frame(mpb, DREAMCAST_ADDR, SUB_PERIPHERAL_1, BLOCK_WRITE, mpb->peripherals[1].data.vmu.vmu_screen_buffer, 50);
            int r = read_maple(mpb);
            mpb->peripherals[1].data.vmu.vmu_refresh = false;
            struct mp_rx_frame * rx_frame2 = (struct mp_rx_frame *)mpb->rx_buffer;
            if (rx_frame2->command == PLEASE_SEND_AGAIN_RESPONSE)
                DEBUG_PRINT("PLEASE_SEND_AGAIN\r\n");
            if (r == MP_CRC_MISMATCH)
            {
                DEBUG_PRINT("CRC MISMATCH\r\n");
                send_frame(mpb, MAIN_PERIPHERAL, DREAMCAST_ADDR, PLEASE_SEND_AGAIN_RESPONSE, NULL, 0);
                continue;
            }
        }

        // Request current button state from controller
        uint32_t get_condition_payload = __builtin_bswap32((uint32_t)FUNC_CONTROLLER);
        send_frame(mpb, DREAMCAST_ADDR, MAIN_PERIPHERAL, GET_CONDITION, (uint8_t *)&get_condition_payload, 1);
        int r = read_maple(mpb);
        if (r == MP_CRC_MISMATCH) {
            DEBUG_PRINT("CRC MISMATCH\r\n");
            send_frame(mpb, MAIN_PERIPHERAL, DREAMCAST_ADDR, PLEASE_SEND_AGAIN_RESPONSE, NULL, 0);
            continue;
        }

        if (r == MP_TIMEOUT)
        {
            DEBUG_PRINT("MAPLE_READ TIMEOUT IN MAIN LOOP, START SEARCHING FOR MAIN PERIPHERAL AGAIN\r\n");
            mpb->state = MP_DREAMCAST_STATE_SEARCHING_FOR_MAIN_PERIPHERAL;
            mpb->peripheral_address = 0;
            mpb->peripherals[0].address = 0;
            continue;
        }

        struct mp_rx_frame * rx_frame = (struct mp_rx_frame *)mpb->rx_buffer;

        mpb->peripherals[0].address = rx_frame->sender_addr;

        switch (rx_frame->command) {
            case REQUEST_DEVICE_INFO:
                DEBUG_PRINT("REQUEST_DEVICE_INFO\r\n");
                break;
            case GET_CONDITION:
                DEBUG_PRINT("GET_CONDITION\r\n");
                break;
            case GET_MEMORY_INFORMATION:
                DEBUG_PRINT("GET_MEMORY_INFORMATION\r\n");
                break;
            case BLOCK_READ:
                DEBUG_PRINT("BLOCK_READ\r\n");
                break;
            case BLOCK_WRITE:
                DEBUG_PRINT("BLOCK_WRITE\r\n");
                break;
            case GET_LAST_ERROR:
                DEBUG_PRINT("GET_LAST_ERROR\r\n");
                break;
            case REQUEST_EXTENDED_DEVICE_INFO:
                DEBUG_PRINT("REQUEST_EXTENDED_DEVICE_INFO\r\n");
                break;
            case RESET_DEVICE:
                DEBUG_PRINT("RESET_DEVICE\r\n");
                break;
            case SHUTDOWN_DEVICE:
                DEBUG_PRINT("SHUTDOWN_DEVICE\r\n");
                break;
            case DEVICE_INFO_RESPONSE:
                DEBUG_PRINT("DEVICE_INFO_RESPONSE\r\n");
                break;
            case EXTENDED_DEVICE_INFO_RESPONSE:
                DEBUG_PRINT("EXTENDED_DEVICE_INFO_RESPONSE\r\n");
                break;
            case COMMAND_ACK_RESPONSE:
                DEBUG_PRINT("COMMAND_ACK_RESPONSE\r\n");
                break;
            case DATA_TRANSFER_RESPONSE:
                //DEBUG_PRINT("DATA_TRANSFER_RESPONSE\r\n");
                parse_data_transfer_response((struct mp_rx_frame *)mpb->rx_buffer);
                break;
            case SET_CONDITION:
                DEBUG_PRINT("SET_CONDITION\r\n");
                print_frame(rx_frame);
                break;
            case NO_RESPONSE:
                DEBUG_PRINT("NO_RESPONSE\r\n");
                break;
            case FUNCTION_CODE_UNSUPPORTED_RESPONSE:
                DEBUG_PRINT("FUNCTION_CODE_UNSUPPORTED_RESPONSE\r\n");
                break;
            case UNKNOWN_COMMAND_RESPONSE:
                DEBUG_PRINT("UNKNOWN_COMMAND_RESPONSE\r\n");
                break;
            case PLEASE_SEND_AGAIN_RESPONSE:
                DEBUG_PRINT("PLEASE_SEND_AGAIN_RESPONSE\r\n");
                break;
            case FILE_ERROR_RESPONSE:
                DEBUG_PRINT("FILE_ERROR_RESPONSE not implemented\r\n");
                break;
            default:
                DEBUG_PRINT("UNKNOWN COMMAND CODE\r\n");
                break;
        }

        mpb->rx_buffer[0] = 0;
        pio_sm_clear_fifos(mpb->rx_pio.pio, mpb->rx_pio.sm);
        sleep_ms(10);
    }
}