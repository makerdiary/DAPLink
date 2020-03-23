/**
 * @file    m2dock.c
 * @brief   board file for M.2 Dock which supports nRF52840 M.2 module
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2018, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_family.h"
#include "target_board.h"

extern const target_cfg_t target_device_nrf52840_256;

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "1028",
    .family_id = kNordic_Nrf52_FamilyID,
    .daplink_url_name = "PRODINFOHTM",
    .daplink_drive_name = "M2-DOCK",
    .daplink_target_url = "https://wiki.makerdiary.com/m2-dock/",
    .target_cfg = &target_device_nrf52840_256,
};
