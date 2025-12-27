/**
 * mp_peripheral.h
 *
 * Handle Maple stuff for Dreamcast peripherals
 *
 * Copyright (c) 2025 Colin Luoma
 */

#ifndef DREAMWAVE_MP_PERIPHERAL_H
#define DREAMWAVE_MP_PERIPHERAL_H

#define MP_PERIPHERAL_DATA_SIZE 512

#include <stdint.h>

typedef enum {
    MP_PERIPHERAL_EMPTY,
    MP_PERIPHERAL_CONTROLLER,
    MP_PERIPHERAL_VMU,
    MP_PERIPHERAL_RUMBLE_PAK
} mp_peripheral_type_t;

enum mp_function_code {
    FUNC_NONE           = 0x000,
    FUNC_CONTROLLER     = 0x001,
    FUNC_MEMORY_CARD    = 0x002,
    FUNC_LCD_DISPLAY    = 0x004,
    FUNC_CLOCK          = 0x008,
    FUNC_MICROPHONE     = 0x010,
    FUNC_AR_GUN         = 0x020,
    FUNC_KEYBOARD       = 0x040,
    FUNC_LIGHT_GUN      = 0x080,
    FUNC_RUMBLE_PAK     = 0x100,
    FUNC_MOUSE          = 0x200
};

typedef struct __attribute__((packed)) {
    uint32_t func;                  // function codes supported by this peripheral (or:ed together) (big endian)
    uint32_t function_data[3];      // additional info for the supported function codes (3 max) (big endian)
    uint8_t area_code;              // regional code of peripheral
    uint8_t connector_direction;    // physical orientation of bus connection
    unsigned char product_name[30];          // name of peripheral
    unsigned char product_license[60];       // license statement
    uint16_t standby_power;         // standby power consumption (little endian)
    uint16_t max_power;             // maximum power consumption (little endian)
} mp_device_info_t;

typedef struct {
    uint8_t pad[MP_PERIPHERAL_DATA_SIZE];
} mp_peripheral_data_controller;
static_assert(sizeof(mp_peripheral_data_controller) == MP_PERIPHERAL_DATA_SIZE);

typedef struct {
    bool vmu_refresh;
    uint8_t vmu_screen_buffer[200];
    uint8_t pad[MP_PERIPHERAL_DATA_SIZE-200-1];
} mp_peripheral_data_vmu;
static_assert(sizeof(mp_peripheral_data_vmu) == MP_PERIPHERAL_DATA_SIZE);

typedef struct {
    mp_peripheral_type_t type;
    mp_device_info_t di;
    uint8_t address;
    union {
        mp_peripheral_data_controller controller;
        mp_peripheral_data_vmu vmu;
    } data;
} mp_peripheral;

/**
 * Initialize all fields in an mp_peripheral
 */
void mp_peripheral_init(mp_peripheral * p);

/**
 * Populates a peripheral's mp_device_info from Maple GET_INFORMATION response data
 */
void mp_peripheral_parse_req_device_info_resp(mp_peripheral * p, uint8_t address, uint8_t * resp, uint32_t len);



#endif //DREAMWAVE_MP_PERIPHERAL_H