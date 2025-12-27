/**
 * dw_hid_descriptors.h
 *
 * HID descriptors for different use cases
 *
 * Copyright (c) 2024 Colin Luoma
 */

#ifndef DREAMWAVE_DW_HID_DESCRIPTORS_H
#define DREAMWAVE_DW_HID_DESCRIPTORS_H

#define DW_HID_DESCRIPTOR_BASIC_SIZE 46
#define DW_HID_DESCRIPTOR_XBOX_SIZE  334

extern const uint8_t dw_hid_descriptor_basic[DW_HID_DESCRIPTOR_BASIC_SIZE];
extern const uint8_t dw_hid_descriptor_xbox[DW_HID_DESCRIPTOR_XBOX_SIZE];

#endif //DREAMWAVE_DW_HID_DESCRIPTORS_H
