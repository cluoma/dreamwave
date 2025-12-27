/**
 * dw_bt_tlv.h
 *
 * Interface for storing data in btstack's tlv, e.g. bluetooth addresses
 *
 * Copyright (c) 2024 Colin Luoma
 */

#ifndef DREAMWAVE_DW_BT_TLV_H
#define DREAMWAVE_DW_BT_TLV_H

#include <stdint.h>

typedef enum {
    DW_TLV_TAG_NONE,
    DW_TLV_TAG_LAST_CONNECTED_BTADDR
} dw_bt_tlv_tag_t;

void dw_bt_tlv_init();
int dw_bt_tlv_get_tag(dw_bt_tlv_tag_t tag, uint8_t * buf);
int dw_bt_tlv_set_tag(dw_bt_tlv_tag_t tag, uint8_t * buf);

#endif //DREAMWAVE_DW_BT_TLV_H