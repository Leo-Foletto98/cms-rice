
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <ctype.h>
#include <iostream>
#include <vector>
#include <dirent.h>
#include <cstdint>
#include <cstring>
#include <cstddef>

#include "libraries/esp32-modbus/modbus.h"
#include "libraries/esp32-modbus/modbus.cpp"

extern "C" {


    #include "freertos/FreeRTOS.h"
    #include "freertos/queue.h"
    #include "freertos/task.h"
    #include "freertos/event_groups.h"
    #include "driver/temp_sensor.h"
    #include "driver/gpio.h"

    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_wifi.h"
    #include "esp_netif.h"

    #include "esp_utils.h"
    #include "esp_storage.h"
    #include "espnow.h"

    #include "esp_sleep.h"
}

MODBUS mdb;

//Set the same Wi-Fi channel of the ESP Master
#define WIFI_CHANNEL 1


static const char *TAG = "slave-esp-now";


static void wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_promiscuous(1);

    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    esp_wifi_set_promiscuous(0);

    ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );

}

static void send_temperature_task(void *arg)
{
    ESP_LOGI(TAG, "Temperature read handle task is running");

    esp_err_t ret  = ESP_OK;
    uint32_t count = 0;
    float tsens_out;
    char temperature[32] = {0};

    espnow_frame_head_t frame_head = {
        .broadcast        = true,
        .retransmit_count = 5,  
    };
    

    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
   

    mdb.init(UART_NUM_1, 115200, 256, 2, 2, 19, 18); //Inicia o modbus

    gpio_reset_pin(GPIO_NUM_7);

    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_7, 1);

    int32_t leitura = mdb.request(1, 3, 10);
    float distance = float(leitura)/10;

    ESP_LOGW(__func__, "Sensor 1: [Distance=%.2f] ", distance);

    while(1){

        leitura = mdb.request(1, 3, 10);
        distance = float(leitura)/10;

        ESP_LOGW(__func__, "Sensor 1: [Distance=%.2f] ", distance);

        if(leitura != INT32_MIN){

            temp_sensor_read_celsius(&tsens_out);

            sprintf(temperature,"%.2f,%.2f",tsens_out,distance );

            ret = espnow_send(ESPNOW_TYPE_DATA, ESPNOW_ADDR_BROADCAST, temperature, sizeof(temperature), &frame_head, (5000 / portTICK_RATE_MS));
            ESP_ERROR_CONTINUE(ret != ESP_OK, "<%s> espnow_send", esp_err_to_name(ret));

            ESP_LOGI(TAG, "espnow_send, count: %d, size: %d, data: %s", count++, strlen(temperature), temperature);

        }else{


            temp_sensor_read_celsius(&tsens_out);

            sprintf(temperature,"%.2f",tsens_out);

            ret = espnow_send(ESPNOW_TYPE_DATA, ESPNOW_ADDR_BROADCAST, temperature, sizeof(temperature), &frame_head, (5000 / portTICK_RATE_MS));
            ESP_ERROR_CONTINUE(ret != ESP_OK, "<%s> espnow_send", esp_err_to_name(ret));

            ESP_LOGI(TAG, "espnow_send, count: %d, size: %d, data: %s", count++, strlen(temperature), temperature);

        }

        /*
        while (count < 10)
        {
            
            leitura = mdb.request(1, 3, 10);
            distance = float(leitura)/10;

            ESP_LOGW(__func__, "Sensor 1: [Distance=%.2f] ", distance);

            if(leitura != INT32_MIN){


                temp_sensor_read_celsius(&tsens_out);

                sprintf(temperature,"%.2f,%.2f",tsens_out,distance );

                ret = espnow_send(ESPNOW_TYPE_DATA, ESPNOW_ADDR_BROADCAST, temperature, sizeof(temperature), &frame_head, (5000 / portTICK_RATE_MS));
                ESP_ERROR_CONTINUE(ret != ESP_OK, "<%s> espnow_send", esp_err_to_name(ret));

                ESP_LOGI(TAG, "espnow_send, count: %d, size: %d, data: %s", count++, strlen(temperature), temperature);

                vTaskDelay(100 / portTICK_RATE_MS);

                ESP_LOGW(__func__, "Entering in deep sleep ");

                gpio_set_level(GPIO_NUM_7, 0);

                vTaskDelay(1000 / portTICK_RATE_MS);

                esp_sleep_enable_timer_wakeup(60 * 1000000);

                esp_deep_sleep_start();

            }

            vTaskDelay(100 / portTICK_RATE_MS);

            count++;
        }

        ESP_LOGW(__func__, "Error to read sensor, sleeping ");

        gpio_set_level(GPIO_NUM_7, 0);

        vTaskDelay(1000 / portTICK_RATE_MS);

        esp_sleep_enable_timer_wakeup(60 * 1000000);

        esp_deep_sleep_start();

        

        */

        

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Temperature handle task is exit");

    vTaskDelete(NULL);
}

extern "C" void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();
    esp_event_loop_create_default();

    wifi_init();

    espnow_config_t espnow_config = {};

    uint8_t pmk[16] = {'E','S','P','_','N','O','W'};

    memcpy(espnow_config.pmk,pmk,sizeof(espnow_config.pmk));
    espnow_config.forward_enable = true;
    espnow_config.send_retry_num = 10;
    espnow_config.send_max_timeout = pdMS_TO_TICKS(3000);
    espnow_config.qsize.data      = 64;
    espnow_config.qsize.ack = 8;
    espnow_config.qsize.forward = 8;
    espnow_config.qsize.group = 8;
    espnow_config.qsize. provisoning = 0;
    espnow_config.qsize.control_bind = 0;
    espnow_config.qsize.control_data = 0;
    espnow_config.qsize.ota_status = 0;
    espnow_config.qsize.ota_data = 0;
    espnow_config.qsize. debug_log = 0;
    espnow_config.qsize.debug_command = 0;
    espnow_config.qsize.reserved = 0;
    
    espnow_init(&espnow_config);

    xTaskCreate(send_temperature_task, "send_temperature_task", 4 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

}   
