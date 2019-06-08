
#include <string.h>

#include "aws_clientcredential.h"
#include "aws_mqtt_agent.h"

#include "esp_log.h"
#include "driver/uart.h"

/* local header */
#include "mqtt_publisher.h"
#include "mhz19b_task.h"

#define UART_PORT UART_NUM_2
#define BUF_SIZE (1024)

#define UINT16_MAX_PLACES (5)

static const TickType_t LOOP_FREQUENCY = 60000 / portTICK_RATE_MS;
static const TickType_t MAX_MQTT_TIMEOUT = 1000 / portTICK_RATE_MS;
static const char *TAG = "MH-Z19B";

static uint8_t rx_buffer[BUF_SIZE];

#pragma pack(1)
typedef struct
{
    uint8_t start_byte;
    uint8_t sensor;
    uint8_t command;
    uint8_t payload[5];
    char checksum;
} mhz19b_request_data_t;
#pragma pack()

#pragma pack(1)
typedef union
{
    mhz19b_request_data_t data;
    char data_bytes[9];
} mhz19b_request_t;
#pragma pack()

static void publish_sensor_data(MqttPublisherDataHandle_t publishDataHandle, int concentration)
{
    char valueStr[UINT16_MAX_PLACES];
    snprintf(valueStr, sizeof(valueStr), "%d", concentration);
    UpdatePublishData(publishDataHandle, "co2 conc", valueStr);
}

static char get_checksum(char *packet)
{
    char checksum = 0;
    for(int i = 1; i < 8; i++) {
        checksum += packet[i];
    }
    checksum = ~checksum;
    checksum += 1;
    return checksum;
}

static void create_read_concentration_request(mhz19b_request_t* request)
{
    request->data.start_byte = 0xFF;
    request->data.sensor = 0x01;
    request->data.command = 0x86;
    memset(request->data.payload, 0x00, sizeof(request->data.payload));
    request->data.checksum = get_checksum(request->data_bytes);
}

static esp_err_t write_request(const mhz19b_request_t* request)
{
    return uart_write_bytes(UART_PORT, request->data_bytes, sizeof(request->data_bytes));
}

static esp_err_t read_concentration_response(uint16_t* concentration)
{
    int rslt = uart_read_bytes(UART_PORT, rx_buffer, sizeof(rx_buffer), 1000 / portTICK_RATE_MS);
    if (rslt != 9) {
        ESP_LOGI(TAG, "failed to read bytes, invalid data size");
        return ESP_FAIL;
    }

    if (get_checksum((char*) rx_buffer) != (char) rx_buffer[8]) {
        ESP_LOGI(TAG, "failed to read bytes, invalid checksum");
        return ESP_FAIL;
    }

    (*concentration) = (rx_buffer[2] << 8) | rx_buffer[3];
    return ESP_OK;
}

void mhz19b_task(void *arg)
{
    ESP_LOGI(TAG, "initializing UART driver");

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    if (ESP_OK != uart_param_config(UART_PORT, &uart_config)) {
        ESP_LOGI(TAG, "failed to set UART parameter");
        return;
    }
    
    if (ESP_OK != uart_set_pin(UART_PORT, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE))
    {
        ESP_LOGI(TAG, "failed to set UART pin");
        return;
    }

    if (ESP_OK != uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0)) {
        ESP_LOGI(TAG, "failed to install UART driver");
        return;
    }

    ESP_LOGI(TAG, "create publish data handle");
    MqttPublisherDataHandle_t publishDataHandle = CreatePublishDataHandle("mh-z19b");

    mhz19b_request_t read_co2_request;
    create_read_concentration_request(&read_co2_request);    

    ESP_LOGI(TAG, "start loop");

    int rslt = 0;
    uint16_t concentration = 0;

    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        rslt = write_request(&read_co2_request);
        if (rslt < 0) {
            ESP_LOGI(TAG, "failed to write bytes to UART port");
            goto next_loop;
        }

        rslt = read_concentration_response(&concentration);
        if (rslt < 0) {
            ESP_LOGI(TAG, "faild to read bytes from UART port");
            goto next_loop;
        }

        ESP_LOGI(TAG, "co2: %d", concentration);
        publish_sensor_data(publishDataHandle, concentration);

next_loop:
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, LOOP_FREQUENCY);
    }

    DeletePublishDataHandle(publishDataHandle);
    uart_driver_delete(UART_PORT);

    vTaskDelete(NULL);
}