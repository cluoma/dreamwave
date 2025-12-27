/**
 * dw_bt_hid.h
 *
 * Handle all things Bluetooth and HID
 *
 * Copyright (c) 2024 Colin Luoma
 */

#ifndef DW_BT_HID_H
#define DW_BT_HID_H

typedef enum {
    DW_BT_STATE_IDLE,
    DW_BT_STATE_RECONNECTING,
    DW_BT_STATE_PAIRING,
    DW_BT_STATE_CONNECTED
} dw_bt_connection_state_t;

/**
 * Initialize BTStack and configure for HID. Run BTStack.
 */
int dw_bt_init();

/**
 * Get current state of the controller
 */
dw_bt_connection_state_t dw_bt_connection_state();

#endif //DW_BT_HID_H
