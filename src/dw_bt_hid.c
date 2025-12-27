/**
 * dw_bt_hid.c
 *
 * Handle all things Bluetooth and HID
 *
 * Modified from https://github.com/bluekitchen/btstack/blob/master/example/hid_keyboard_demo.c
 * Copyright (C) 2014 BlueKitchen GmbH
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "btstack.h"
#include "dw_bt_hid.h"
#include "maple.h"
#include "dw_config.h"
#include "dw_bt_tlv.h"
#include "dw_hid_descriptors.h"
#include "utils.h"

#define HID_NO_BOOT_MODE false
#define HID_YES_BOOT_MODE true

// variables from maple
extern mp_dreamcast_t * mpb;
extern controller_condition_t cond;

static const char hid_device_name[] = DW_CONTROLLER_NAME;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static bd_addr_t host_addr;                 // last connected-to BT host
static bool found_last_host_addr = false;   // do we have a previous BT host address? Either in TLV or from
                                            // a new connection
// working buffers
static uint8_t hid_service_buffer[720];
static uint8_t device_id_sdp_service_buffer[100];

// HID connection ID
static uint16_t hid_cid;

// Connection states
static dw_bt_connection_state_t state = DW_BT_STATE_IDLE;
static uint8_t reconnect_attempts = 0;
static btstack_timer_source_t reconnect_timer;


/**
 * Send HID controller report, populated with data from the latest Maple Controller Condition
 */
void send_controller_report()
{
    uint8_t report[] = {
        (uint8_t)0xA1,  // 0xA1 Input Report
        (uint8_t)0x01,  // 0x01 Report ID
        (uint8_t)cond.joy_x,
        (uint8_t)cond.joy_y,
        (uint8_t)cond.trigger_right,
        (uint8_t)cond.trigger_left,
        (uint8_t) (cond.a | (cond.b << 1) | (cond.x << 2) | (cond.y << 3) | (cond.d_up << 4) | (cond.d_down << 5) | (cond.d_left << 6) | (cond.d_right << 7)),
        (uint8_t) cond.start
    };
    hid_device_send_interrupt_message(hid_cid, &report[0], sizeof(report));
}


void start_pairing()
{
    DEBUG_PRINT("Starting pairing\r\n");
    state = DW_BT_STATE_PAIRING;
    reconnect_attempts = 0;
    gap_inquiry_start(2);
    gap_discoverable_control(1);
}

void hid_try_reconnect()
{
    if (!found_last_host_addr) {
        DEBUG_PRINT("No previous host address available\r\n");
        start_pairing();
        return;
    }
    if (reconnect_attempts >= DW_RECONNECT_ATTEMPTS) {
        DEBUG_PRINT("Max reconnect attempts reached\r\n");
        reconnect_attempts = 0;
        start_pairing();
        return;
    }

    DEBUG_PRINT("Trying reconnect to %s, attempt %d of %d\r\n", bd_addr_to_str(host_addr), reconnect_attempts + 1, DW_RECONNECT_ATTEMPTS);
    state = DW_BT_STATE_RECONNECTING;
    reconnect_attempts++;
    hid_device_connect(host_addr, &hid_cid);
}

void hid_reconnect_timer_handler(btstack_timer_source_t * ts)
{
    UNUSED(ts);
    hid_try_reconnect();
}

void hid_schedule_reconnect(void)
{
    if (reconnect_attempts >= DW_RECONNECT_ATTEMPTS) {
        DEBUG_PRINT("Max reconnect attempts reached\r\n");
        reconnect_attempts = 0;
        start_pairing();
        return;
    }

    uint32_t delay = DW_RECONNECT_DELAY_MS + (reconnect_attempts * DW_RECONNECT_BACKOFF_MS);
    DEBUG_PRINT("Reconnect scheduled in %lu ms\n", delay);
    btstack_run_loop_set_timer(&reconnect_timer, delay);
    btstack_run_loop_set_timer_handler(&reconnect_timer, &hid_reconnect_timer_handler);
    btstack_run_loop_add_timer(&reconnect_timer);
}

/**
 * Bluetooh HCI and HID event packet handler
 */
static void hci_hid_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t packet_size)
{
    UNUSED(channel);
    UNUSED(packet_size);

    switch (packet_type)
    {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet))
            {
                case BTSTACK_EVENT_STATE:
                    // Immediately try to reconnect to the last saved bd_addr when btstack is ready
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
                    {
                        if (found_last_host_addr)
                        {
                            DEBUG_PRINT("BTstack working to %s\r\n", bd_addr_to_str(host_addr));
                            hid_schedule_reconnect();
                        }
                        else
                        {
                            DEBUG_PRINT("No stored host, starting in pairing mode\r\n");
                            start_pairing();
                        }
                    }
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // btstack autoaccepts SSP
                    break;

                case HCI_EVENT_CONNECTION_COMPLETE:
                    // not used atm
                    break;

                case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT:
                    if (hci_event_authentication_complete_get_status(packet) != ERROR_CODE_SUCCESS)
                    {
                        DEBUG_PRINT("Authentication failed, clearing stored key\r\n");
                        gap_drop_link_key_for_bd_addr(host_addr);
                    }
                    break;

                case HCI_EVENT_HID_META:
                    switch (hci_event_hid_meta_get_subevent_code(packet))
                    {
                        case HID_SUBEVENT_CONNECTION_OPENED:
                            DEBUG_PRINT("HID connection opened\n");
                            if (hid_subevent_connection_opened_get_status(packet) == ERROR_CODE_SUCCESS)
                            {
                                DEBUG_PRINT("HID connection success\r\n");
                                state = DW_BT_STATE_CONNECTED;
                                reconnect_attempts = 0;

                                // get bd_addr and hid connection id
                                hid_subevent_connection_opened_get_bd_addr(packet, host_addr);
                                found_last_host_addr = true;
                                hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);

                                // Save the bd_addr for a quick-connect later
                                if (dw_bt_tlv_set_tag(DW_TLV_TAG_LAST_CONNECTED_BTADDR, host_addr))
                                    DEBUG_PRINT("Unable to save host bd_addr_t\r\n");

                                // send reports immediately
                                hid_device_request_can_send_now_event(hid_cid);
                                gap_discoverable_control(0);
                            }
                            else
                            {
                                switch (state)
                                {
                                    case DW_BT_STATE_RECONNECTING:
                                        DEBUG_PRINT("HID connection failed.. scheduling reconnect\r\n");
                                        hid_schedule_reconnect();
                                        break;
                                    case DW_BT_STATE_PAIRING:
                                        // Pairing attempt failed
                                        DEBUG_PRINT("HID connection failed during pairing attempt\r\n");
                                        break;
                                    default:
                                        break;
                                }
                            }
                            break;

                        case HID_SUBEVENT_CONNECTION_CLOSED:
                            DEBUG_PRINT("HID disconnected\n");
                            hid_cid = 0;
                            state = DW_BT_STATE_IDLE;
                            // Start reconnection attempts
                            reconnect_attempts = 0;
                            hid_schedule_reconnect();
                            break;
                        case HID_SUBEVENT_CAN_SEND_NOW:
                            // send HID report and queue another send
                            send_controller_report();
                            hid_device_request_can_send_now_event(hid_cid);
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

/**
 * Bluetooth custom L2CAP packet handler
 */
static void l2cap_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    static uint16_t l2cap_cid = 0;
    switch (packet_type)
    {
        case L2CAP_DATA_PACKET:
            // right now the only thing arriving here should be a vmu lcd packet
            if (size == 192)
            {
                mp_push_vmu_screen_data(mpb, packet, 192);
            }
            break;

        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet))
            {
                case L2CAP_EVENT_INCOMING_CONNECTION:
                    if (l2cap_cid != 0)
                        return;
                    l2cap_cid = l2cap_event_incoming_connection_get_local_cid(packet);
                    l2cap_accept_connection(l2cap_cid);
                    break;

                case L2CAP_EVENT_CHANNEL_OPENED:
                    if (l2cap_event_channel_opened_get_status(packet) == ERROR_CODE_SUCCESS)
                    {
                        l2cap_cid = l2cap_event_channel_opened_get_local_cid(packet);
                        printf("L2CAP channel opened, cid 0x%02x\n", l2cap_cid);
                    }
                    else
                    {
                        DEBUG_PRINT("L2CAP connection failed\n");
                    }
                    break;

                case L2CAP_EVENT_CHANNEL_CLOSED:
                    DEBUG_PRINT("L2CAP channel closed\n");
                    l2cap_cid = 0;
                    break;

                default:
                    break;
            }
        default:
            break;
    }
}

/**
 * Initialize BTStack and configure for HID. Run BTStack.
 */
int dw_bt_init()
{
    dw_bt_tlv_init();
    if (dw_bt_tlv_get_tag(DW_TLV_TAG_LAST_CONNECTED_BTADDR, host_addr) > 0)
    {
        DEBUG_PRINT("Found saved bd_addr %s\r\n", bd_addr_to_str(host_addr));
        found_last_host_addr = true;
    }

    // GAP
    gap_discoverable_control(0);
    gap_set_class_of_device(0x2508);    // 0x2508 is a generic gamepad
    gap_set_local_name(DW_CONTROLLER_NAME);  // name of the device
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    // L2CAP
    l2cap_init();

    // SDP
    sdp_init();

    // HID
    hid_sdp_record_t hid_params = {
        .hid_device_subclass = 0x2508,  // 0x2508 is a gamepad
        .hid_country_code = 33,         // US
        .hid_virtual_cable = false,
        .hid_remote_wake = true,
        .hid_reconnect_initiate = true,
        .hid_normally_connectable = true,
        .hid_boot_device = false,
        .hid_ssr_host_max_latency = 0xFFFF,
        .hid_ssr_host_min_timeout = 0xFFFF,
        .hid_supervision_timeout = 3200,
        .hid_descriptor = dw_hid_descriptor_basic,
        .hid_descriptor_size = DW_HID_DESCRIPTOR_BASIC_SIZE,
        .device_name = hid_device_name
    };

    // Add HID to the SDP records
    memset(hid_service_buffer, 0, sizeof(hid_service_buffer));
    hid_create_sdp_record(hid_service_buffer, sdp_create_service_record_handle(), &hid_params);
    btstack_assert(de_get_len(hid_service_buffer) <= sizeof(hid_service_buffer));
    sdp_register_service(hid_service_buffer);

    // Add VID/PID data to the SDP records
    device_id_create_sdp_record(device_id_sdp_service_buffer, sdp_create_service_record_handle(), DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, DW_VENDOR_ID, DW_PRODUCT_ID, 1);
    btstack_assert(de_get_len(device_id_sdp_service_buffer) <= sizeof(device_id_sdp_service_buffer));
    sdp_register_service(device_id_sdp_service_buffer);

    // HID init, add descriptor
    hid_device_init(HID_NO_BOOT_MODE, DW_HID_DESCRIPTOR_BASIC_SIZE, dw_hid_descriptor_basic);

    // register for HCI and HID event callbacks
    hci_event_callback_registration.callback = &hci_hid_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    // HID
    hid_device_register_packet_handler(&hci_hid_packet_handler);

    // Register custom L2CAP channel for data exchange
    if (l2cap_register_service(&l2cap_packet_handler, DW_CUSTOM_L2CAP_PSM, DW_CUSTOM_L2CAP_MTU, gap_get_security_level()) != ERROR_CODE_SUCCESS)
    {
        printf("Failed to register custom L2CAP\r\n");
    }

    hci_power_control(HCI_POWER_ON);

    return 0;
}

dw_bt_connection_state_t dw_bt_connection_state()
{
    return state;
}