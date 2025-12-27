/**
 * dw_config.h
 *
 * Configuration defines for Dreamwave project
 *
 * Copyright (c) 2024 Colin Luoma
 */

#ifndef DW_CONFIG_H
#define DW_CONFIG_H

#define DW_CONTROLLER_NAME "Dreamwave Controller"

// VID and PID for HID, currently using Bluekitchen for testing
#define DW_VENDOR_ID 0x048F     //BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH
#define DW_PRODUCT_ID 0x0002
// #define DW_VENDOR_ID 0x045E     //Microsoft
// #define DW_PRODUCT_ID 0x02E0    // Xbox One Bluetooth

#define DW_CUSTOM_L2CAP_PSM 0x1003
#define DW_CUSTOM_L2CAP_MTU 1024 // number of bytes, currently only sending vmu screen data (192 bytes)

// Reconnect configuration
#define DW_RECONNECT_ATTEMPTS 3
#define DW_RECONNECT_DELAY_MS 0
#define DW_RECONNECT_BACKOFF_MS 2000

#endif //DW_CONFIG_H
