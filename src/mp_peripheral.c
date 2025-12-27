/**
 * mp_peripheral.c
 *
 * Handle Maple stuff for Dreamcast peripherals
 *
 * Copyright (c) 2025 Colin Luoma
 */

#include <stdio.h>
#include <string.h>

#include "mp_peripheral.h"
#include "utils.h"

void mp_peripheral_init(mp_peripheral * p)
{
    p->type = MP_PERIPHERAL_EMPTY;
    p->di.func = 0;
    p->address = 0;
    memset(&p->data, 0, MP_PERIPHERAL_DATA_SIZE);
    // switch (p->type)
    // {
    //     case MP_PERIPHERAL_CONTROLLER:
    //         p->data = NULL;
    //         break;
    //     case MP_PERIPHERAL_VMU:
    //         p->data = &vmu1;
    //         mp_peripheral_vmu_init(&vmu1);
    //         break;
    //     case MP_PERIPHERAL_RUMBLE_PAK:
    //         break;
    //     default:
    //         break;
    // }
}

/**
 * Populates a peripheral's mp_device_info from Maple GET_INFORMATION response data
 */
void mp_peripheral_parse_req_device_info_resp(mp_peripheral * p, uint8_t address, uint8_t * resp, uint32_t len)
{
    // Fix the byte order of the payload
    uint32_t *payload = (uint32_t *)resp;
    for (int i = 0; i < len; i++)
    {
        payload[i] = __builtin_bswap32(payload[i]);
    }

    memcpy(&p->di, payload, sizeof(mp_device_info_t));

    // Fix byte order of certain entries
    p->di.func = __builtin_bswap32(p->di.func);
    p->di.function_data[0] = __builtin_bswap32(p->di.function_data[0]);
    p->di.function_data[1] = __builtin_bswap32(p->di.function_data[1]);
    p->di.function_data[2] = __builtin_bswap32(p->di.function_data[2]);

    p->address = address;

    if (p->di.func & FUNC_CONTROLLER)
    {
        p->type = MP_PERIPHERAL_CONTROLLER;
        DEBUG_PRINT("GOT CONTROLLER\r\n");
        DEBUG_PRINT("\"%.30s\"\r\n", p->di.product_name);
    }
    else if (p->di.func & FUNC_MEMORY_CARD)
    {
        p->type = MP_PERIPHERAL_VMU;
        p->data.vmu.vmu_refresh = false;
        p->data.vmu.vmu_screen_buffer[0] = 0x00;
        p->data.vmu.vmu_screen_buffer[1] = 0x00;
        p->data.vmu.vmu_screen_buffer[2] = 0x00;
        p->data.vmu.vmu_screen_buffer[3] = 0x04;
        p->data.vmu.vmu_screen_buffer[4] = 0x00;
        p->data.vmu.vmu_screen_buffer[5] = 0x00;
        p->data.vmu.vmu_screen_buffer[6] = 0x00;
        p->data.vmu.vmu_screen_buffer[7] = 0x00;
    }
    else if (p->di.func & FUNC_RUMBLE_PAK)
    {
        p->type = MP_PERIPHERAL_RUMBLE_PAK;
    }
}