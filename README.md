| Supported Targets | 	ESP32-C6 	|

---------------------------------------------------------
Warning and disclaimer : 
this code still needs to be improved and cleaned up from the various tests. It is provided for those who want a working basis for using I2C in “light sleep” mode.
---------------------------------------------------------

# CO2/Temperature/humidity sensor

This test code shows how to configure Zigbee end device and use it as a HA CO2 sensor. 
- The ESP32-C6 is wired with a SCD4x sensor from Sensirion.
- The code use light sleep mode to save batteries.
- Mean consumption is approximately 7mA. (This measure is not precise,esp32-c6)
- Mean consumption is approximately 15mA @3.3v or 3.2mA @3.3v in low power periodic mode. (From data sheet,scd40)
- Value are read from the sensor each 2 minutes and a zigbee report is done if the measure is different than the previous.


## Hardware Required

* One development board with ESP32-C6 SoC acting as Zigbee end-device 
* A USB cable for power supply and programming
* A Sensirion SCD4x sensor wired in I2C (SCL --> pin 7 ; SDA --> pin 6)

## Todo
* Disable automatic_self_calibration, automatic_self_calibration automatically reset to lowest value every week on scd4x, 0x21b1
* Enable forced_recalibration on demand, from push button on board , with led flash for confirmation. 0x21ac
* Enable low power periodic measurement mode (1 measurement every 30 seconds) for lower power consumtion. 0x21ac
* Use ULP for i2c readings of scd4x for low power readings

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py set-target esp32-c6` command.

## Erase the NVRAM 

Before flash it to the board, it is recommended to erase NVRAM using `idf.py -p PORT erase-flash`

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.


## Example Output

To enable text output in ligth sleep use menuconfig

Component config  ->  ESP-Driver:USB Serial/JTAG Configuration -> [*] Don't enter the automatic light sleep when USB Serial/JTAG port is connected

As you run the example, you will see the following log:

```
I (478) main_task: Returned from app_main()
I (479) ESP_ZB_CO2: ZDO signal: ZDO Config Ready (0x17), status: ESP_FAIL
I (482) ESP_ZB_CO2: Initialize Zigbee stack
I (3348) ESP_ZB_CO2: Deferred driver initialization successful
I (3351) ESP_ZB_CO2: STOP mesures périodiques OK !
I (3400) ESP_ZB_CO2: Device started up in non factory-reset mode
I (3400) ESP_ZB_CO2: Device rebooted
I (3502) ESP_ZB_CO2: START mesures périodiques OK !
I (70630) ESP_ZB_CO2: ZDO signal: ZDO Device Unavailable (0x3c), status: ESP_OK
I (110764) ESP_ZB_CO2: ZDO signal: ZDO Device Unavailable (0x3c), status: ESP_OK
I (123555) ESP_ZB_CO2: CO2: 824 Hum: 77.7 Tmp: 19.4
I (123555) ESP_ZB_CO2: Set attribute 0 in cluster 1037 succeded
I (123605) ESP_ZB_CO2: Set attribute 0 in cluster 1026 succeded
I (123605) ESP_ZB_CO2: Set attribute 0 in cluster 1029 succeded
I (150899) ESP_ZB_CO2: ZDO signal: ZDO Device Unavailable (0x3c), status: ESP_OK
W (161302) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
W (181173) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
I (243609) ESP_ZB_CO2: CO2: 809 Hum: 77.2 Tmp: 19.5
I (243610) ESP_ZB_CO2: Set attribute 0 in cluster 1037 succeded
I (243660) ESP_ZB_CO2: Set attribute 0 in cluster 1026 succeded
I (243660) ESP_ZB_CO2: Set attribute 0 in cluster 1029 succeded
W (243831) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
W (244048) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
I (363664) ESP_ZB_CO2: CO2: 804 Hum: 76.5 Tmp: 19.4
I (363665) ESP_ZB_CO2: Set attribute 0 in cluster 1037 succeded
I (363715) ESP_ZB_CO2: Set attribute 0 in cluster 1026 succeded
I (363715) ESP_ZB_CO2: Set attribute 0 in cluster 1029 succeded
W (363889) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
W (364265) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
I (483719) ESP_ZB_CO2: CO2: 807 Hum: 75.9 Tmp: 19.4
I (483720) ESP_ZB_CO2: Set attribute 0 in cluster 1037 succeded
I (483770) ESP_ZB_CO2: Set attribute 0 in cluster 1026 succeded
I (483770) ESP_ZB_CO2: Set attribute 0 in cluster 1029 succeded
W (483944) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback
W (484162) ESP_ZB_CO2: Receive Zigbee action(0x1005) callback

```
 
## Pairing with HA box Jeedom 

![alt text](https://github.com/mycael/esp-zigbee-co2/blob/main/jeedom.jpg?raw=true)

