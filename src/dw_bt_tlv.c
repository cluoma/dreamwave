/**
 * dw_bt_tlv.c
 *
 * Interface for storing data in btstack's tlv, e.g. bluetooth addresses
 *
 * Copyright (c) 2024 Colin Luoma
 */

#include <stdio.h>
#include "bluetooth.h"
#include "btstack_tlv.h"
#include "btstack_tlv_flash_bank.h"
#include "dw_bt_tlv.h"
#include "utils.h"

static const btstack_tlv_t* tlv_impl;
static btstack_tlv_flash_bank_t* tlv_context;

void dw_bt_tlv_init()
{
    btstack_tlv_get_instance(&tlv_impl, (void**)&tlv_context);
    if (!tlv_impl || !tlv_context)
    {
        DEBUG_PRINT("No TLV available\r\n");
    }
}

int dw_bt_tlv_get_tag(dw_bt_tlv_tag_t tag, uint8_t * buf)
{
    int r;
    switch (tag)
    {
        case DW_TLV_TAG_LAST_CONNECTED_BTADDR:
            r = tlv_impl->get_tag(tlv_context, DW_TLV_TAG_LAST_CONNECTED_BTADDR, buf, BD_ADDR_LEN);
            break;
        default:
            r = 0;
            break;
    }
    return r;
}

int dw_bt_tlv_set_tag(dw_bt_tlv_tag_t tag, uint8_t * buf)
{
    // returns 0 on success
    int r;
    switch (tag)
    {
        case DW_TLV_TAG_LAST_CONNECTED_BTADDR:
            r = tlv_impl->store_tag(tlv_context, DW_TLV_TAG_LAST_CONNECTED_BTADDR, buf, BD_ADDR_LEN);
            break;
        default:
            r = 0;
            break;
    }
    return r;
}
