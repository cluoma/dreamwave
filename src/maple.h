/**
 * maple.h
 *
 * Handle reading and writing to Sega Dreamcast Maple Bus using PIO
 *
 * Copyright (c) 2024 Colin Luoma
 */

#ifndef MAPLE_H
#define MAPLE_H

#include "hardware/pio.h"
#include "mp_peripheral.h"

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 640

#define MP_MAX_CONNECTED_PERIPHERALS 3

typedef enum {
    MP_DREAMCAST_STATE_INITIALIZING,
    MP_DREAMCAST_STATE_READY,
    MP_DREAMCAST_STATE_SEARCHING_FOR_MAIN_PERIPHERAL,
    MP_DREAMCAST_STATE_RUNNING
} mp_dreamcast_state_t;

typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t x;
    uint8_t y;
    uint8_t start;
    uint8_t select;
    uint8_t d_up;
    uint8_t d_down;
    uint8_t d_left;
    uint8_t d_right;
    uint8_t trigger_right;  // 0-255
    uint8_t trigger_left;   // 0-255
    uint8_t bumper_right;   // 0-1
    uint8_t bumper_left;    // 0-1
    uint8_t joy_x;          // 0-255, 128 midpoint
    uint8_t joy_y;          // 0-255, 128 midpoint
    uint8_t joy_x2;         // 0-255, 128 midpoint
    uint8_t joy_y2;         // 0-255, 128 midpoint
    uint8_t d2_up;
    uint8_t d2_down;
    uint8_t d2_left;
    uint8_t d2_right;
} controller_condition_t;
#define DEFAULT_CONTROLLER_CONDITION {0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,128,128,128,0,0,0,0};

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
} mp_pio_block_t;

typedef struct {
    // Running state
    mp_dreamcast_state_t state;

    // Maple buffers
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];
    mp_pio_block_t rx_pio;
    mp_pio_block_t tx_pio;

    // Sender address for maple
    uint8_t address;
    // Address supplied by main peripheral, this is useful to know how many peripherals are present
    uint8_t peripheral_address;
    // Any connected peripherals, main and sub
    mp_peripheral peripherals[3];
} mp_dreamcast_t;

/**
 *   Initialize Maple bus from the Dreamcast's pov. Get GPIO and PIOs ready, etc.
 */
void mp_dreamcast_init(mp_dreamcast_t * mpb);

/**
 *  Run main Maple loop.
 */
void mp_dreamcast_run(mp_dreamcast_t * mpb);

/**
 *  Push some data to the VMU LCD, triggers a BLOCK_WRITE to the VMU LCD with the data
 */
void mp_push_vmu_screen_data(mp_dreamcast_t * mpb, uint8_t * data, uint32_t len);

#endif //MAPLE_H
