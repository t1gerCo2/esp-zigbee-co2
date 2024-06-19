/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_color_dimmable_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zigbee_core.h"
//#include "light_driver.h"

/* Zigbee configuration */
#define MAX_CHILDREN                    10                                      /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE       false                                   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   4000                                    /* 1000 millisecond (default : 4000)*/
#define HA_CUSTOM_CO2_ENDPOINT          12                                      /* esp light switch device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

/*#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zczr_cfg = {                                                           \
            .max_children = MAX_CHILDREN,                                               \
        },                                                                              \
    }*/

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }


#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }



/*************** Prototypes *********************/
static esp_err_t i2c_master_init(void);
static esp_err_t deferred_driver_init(void);
static esp_err_t esp_zb_power_save_init(void);
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_master_init_alt(void);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static void esp_zb_task(void *pvParameters);
void SCD40_task(void *pvParameters);
void status_management (esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id);