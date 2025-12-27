/**
 * main.c
 *
 * Dreamwave setup and init
 *
 * Copyright (c) 2024 Colin Luoma
 */

#include <stdio.h>
#include <pico.h>
#include <stdlib.h>
#include <pico/cyw43_arch.h>
#include <pico/async_context_poll.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/stdio.h>

#include "maple.h"
#include "dw_bt_hid.h"

mp_dreamcast_t * mpb;
volatile bool core1_ready = false;

void maple_entry()
{
    // Enable core0 to lockout core1 while writing to flash
    flash_safe_execute_core_init();
    core1_ready = true;

    // Start Maple
    mp_dreamcast_init(mpb);
    mp_dreamcast_run(mpb);
}

int main(void)
{
    stdio_init_all();
    printf("Starting Dreamwave. It's thinking.\r\n");

    // Start maple on core1 right away
    mpb = malloc(sizeof(mp_dreamcast_t));
    multicore_reset_core1();
    multicore_launch_core1(maple_entry);
    while (!core1_ready) tight_loop_contents();

    // Setup our context to use with btstack
    async_context_poll_t bt_context;
    async_context_poll_init_with_defaults(&bt_context);
    cyw43_arch_set_async_context(&bt_context.core);

    while (cyw43_arch_init() != 0) {
        printf("failed to initialise cyw43_arch\r\n");
        sleep_ms(100);
    }

    while(mpb->state == MP_DREAMCAST_STATE_INITIALIZING) tight_loop_contents();

    // Run BTStack on core0
    dw_bt_init();

    while(true)
    {
        // Continuously poll for work
        async_context_poll(&bt_context.core);
    }
}
