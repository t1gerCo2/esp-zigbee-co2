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

#include "HA_custom_co2.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif

#include "driver/rtc_io.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_carbon_dioxide_measurement.h"
#include "driver/i2c.h"
//#include "driver/i2c_master.h"
#include "string.h"


/*#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif*/
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

#define TIMER_WAKEUP_TIME_US    (55 * 1000 * 1000)

#define I2C_MASTER_SCL_IO           7      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           6      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SCD40_SENSOR_ADDR                 0x62        /*!< Slave address of the MPU9250 sensor */
#define SCD40_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define SCD40_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define SCD40_RESET_BIT                   7


static const char *TAG = "ESP_ZB_CO2";

bool lock_sleep = false;


/********************* Define functions **************************/

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

static DRAM_ATTR esp_pm_lock_handle_t s_pm_lock;

/**
 * @brief SCD40 initialization and measurement reading task
 */
void SCD40_task(void *pvParameters)
{
    
    uint8_t write_buf[2];
    uint8_t read_buf[9];
    esp_err_t ret;
    esp_zb_zcl_status_t status;


    // Stop periodic measurements in case they were started previously
    write_buf[0]=0x3f;
    write_buf[1]=0x86;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SCD40_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ret != ESP_OK) ESP_LOGW(TAG, "Problème STOP mesures périodiques SCD40...");
    else ESP_LOGI(TAG, "STOP mesures périodiques OK !");

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Starting periodic measurements
    write_buf[0]=0x21;
    write_buf[1]=0xb1;

    // Starting periodic low power measurements
    //write_buf[0]=0x21;
    //write_buf[1]=0xac;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SCD40_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ret != ESP_OK) ESP_LOGW(TAG, "Problème START mesures périodiques SCD40...");
    else ESP_LOGI(TAG, "START mesures périodiques OK !");

    //vTaskDelay(5500 / portTICK_PERIOD_MS);      // wait 5s for the first measurement to be accessible.

    //esp_zb_sleep_enable(true);

    while (1)
    {
        vTaskDelay(120000 / portTICK_PERIOD_MS);        // Pause 2 minute


        write_buf[0]=0xec;
        write_buf[1]=0x05;
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, SCD40_SENSOR_ADDR, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf),  I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        if(ret == ESP_OK)
        {
            uint16_t CO2ppm = ((uint16_t)read_buf[0]<<8) + ((uint16_t)read_buf[1]);
            float Temperature = (((((uint16_t) read_buf[3]<<8) + ((uint16_t) read_buf[4])) * 175.0) / 65535.0) - 45.0;
            float Humidity = ((((uint16_t) read_buf[6]<<8) + ((uint16_t)read_buf[7])) * 100.0) / 65535.0;
            ESP_LOGI(TAG, "CO2: %i Hum: %.1f Tmp: %.1f", CO2ppm, Humidity, Temperature);
            uint16_t zbTemperature = (uint16_t)(Temperature * 100.0);
            uint16_t zbHumidity =  (uint16_t)(Humidity * 100.0);
            float_t zbCO2ppm = (float_t)CO2ppm;

            status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &zbCO2ppm, false);
            status_management(status, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID);
            status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &zbTemperature, false);
            status_management(status, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
            status = esp_zb_zcl_set_attribute_val(HA_CUSTOM_CO2_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &zbHumidity, false);
            status_management(status, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID);

            
        }
        else ESP_LOGW(TAG, "Problème lecture SCD40, code %d", ret);


        
    }
}








/**
 * @brief Zigbee initialization task
 */
static void esp_zb_task(void *pvParameters)
{
    lock_sleep = true;
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_sleep_enable(true);
    //esp_zb_sleep_set_threshold(2000);
    esp_zb_init(&zb_nwk_cfg);

    uint8_t ZCLVersion = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint8_t ApplicationVersion = 0x01;
    uint8_t StackVersion = 0x02;
    uint8_t HWVersion = 0x02;
    uint8_t ManufacturerName[] = {7, 'M', 'I', 'C', 'H', 'A', 'E', 'L'};
    uint8_t ModelIdentifier[] = {12, 'E', 'S', 'P', '3', '2', '-', 'C', '6', ' ', 'C', 'O', '2'};
    uint8_t DateCode[] = {8, '2', '0', '2', '4', '0', '5', '2', '4'};
    uint8_t PowerSource = 0x03; //Batteries

    /* basic cluster create */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &ZCLVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)&ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)&DateCode);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &PowerSource);

    /* power configuration cluster create */
    esp_zb_power_config_cluster_cfg_t power_config_cfg = {
        .main_voltage = 50,
        .main_freq = 0,
        .main_alarm_mask = 0,
        .main_voltage_dwell = 0,
        .main_voltage_max = 50,
        .main_voltage_min = 30,
    };
    esp_zb_attribute_list_t *esp_zb_power_config_cluster = esp_zb_power_config_cluster_create(&power_config_cfg);


    /* identify cluster create */
    uint16_t IdentifyTime = 0x0000;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &IdentifyTime);


    /* group cluster create */
    uint8_t NameSupport = 0x00;
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &NameSupport);


    /* carbon dioxide meas cluster create */
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t carbon_dioxide_meas_cfg = {
        .measured_value = 500,
        .min_measured_value = 0,
        .max_measured_value = 100000,
    };
    esp_zb_attribute_list_t *esp_zb_carbon_dioxide_meas_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&carbon_dioxide_meas_cfg);

    /* temperature meas cluster create */
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 2000,
        .min_value = -5000,
        .max_value = 10000,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);
   
    /* humidity meas cluster create */
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
        .measured_value = 5000,
        .min_value = 0,
        .max_value = 10000,
    };
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster = esp_zb_humidity_meas_cluster_create(&humidity_meas_cfg);


    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(esp_zb_cluster_list, esp_zb_carbon_dioxide_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    /* add created endpoint (cluster_list) to endpoint list */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_CUSTOM_CO2_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
	
	esp_zb_device_register(esp_zb_ep_list);


    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}


/**
 * @Sleep configuration brief
 */
static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_LIGHT_SLEEP,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
static esp_err_t i2c_master_init_alt(void)
{
 i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    return i2c_new_master_bus(&i2c_bus_config, &bus_handle);
}
*/

/**
 * @brief Initialisation des drivers
 */
static esp_err_t deferred_driver_init(void)
{
    esp_err_t ret;
    ret = i2c_master_init();
    //ret = i2c_master_init_alt();
    if(ret != ESP_OK) ESP_LOGW(TAG, "Problème initialisation I2C");
    xTaskCreate(SCD40_task, "SCD40_task", 4096, NULL, 5, NULL);
    return ret;
}


static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) 
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } 
            else 
            {
                ESP_LOGI(TAG, "Device rebooted");
                
            }
        } 
        else 
        {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) 
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

        } 
        else 
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) 
            {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } 
            else 
            {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        //ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        switch (message->info.cluster) 
        {
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{

    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}




/**
 * @brief Managing return status for updating attributes
 */
void status_management (esp_zb_zcl_status_t status, uint16_t cluster_id, uint16_t attr_id)
{
    switch(status)
    {
        case 0:
            ESP_LOGI(TAG, "Set attribute %i in cluster %i succeded", attr_id, cluster_id);
            break;
        default:
            ESP_LOGW(TAG, "Set attribute %i in cluster %i failed", attr_id, cluster_id);
            break;
    }
}


