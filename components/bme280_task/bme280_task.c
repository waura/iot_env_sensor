
#define BME280_FLOAT_ENABLE

#include <string.h>

#include "aws_clientcredential.h"
#include "aws_mqtt_agent.h"

#include "esp_log.h"
#include "driver/i2c.h"

/* local header */
#include "driver/bme280_defs.h"
#include "driver/bme280.h"
#include "driver/bme280.c"
#include "mqtt_publisher.h"
#include "bme280_task.h"

#define FLOAT_MAX_PLACES 10

#define I2C_PORT_NUMBER 0

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1 /*!< I2C nack value */

static const TickType_t LOOP_FREQUENCY = 60000 / portTICK_RATE_MS;
static const TickType_t MAX_MQTT_TIMEOUT = 1000 / portTICK_RATE_MS;
static const char *TAG = "BME280";

void user_delay_ms(uint32_t msec)
{
    vTaskDelay(msec / portTICK_RATE_MS);
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (len == 0) {
        return 0;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);  
    if (ret != ESP_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (id << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUMBER, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

static void print_sensor_data(uint32_t task_idx, struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
    ESP_LOGI(TAG, "TASK[%d] temp %0.2f, p %0.2f, hum %0.2f", task_idx, comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
    ESP_LOGI(TAG, "TASK[%d] temp %d, p %d, hum %d", task_idx, comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

static void print_sensor_settings(uint32_t task_idx, struct bme280_dev* dev)
{
    int8_t rslt;

    rslt = bme280_get_sensor_settings(dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "TASK[%d] failed to get BME280 settings", task_idx);
        return;
    }
    ESP_LOGI(TAG, "TASK[%d] BME280 osr_h = %x", task_idx, dev->settings.osr_h);
    ESP_LOGI(TAG, "TASK[%d] BME280 osr_p = %x", task_idx, dev->settings.osr_p);
    ESP_LOGI(TAG, "TASK[%d] BME280 osr_t = %x", task_idx, dev->settings.osr_t);
    ESP_LOGI(TAG, "TASK[%d] BME280 filter = %x", task_idx, dev->settings.filter);
    ESP_LOGI(TAG, "TASK[%d] BME280 standby time = %x", task_idx, dev->settings.standby_time);
}

static void publish_sensor_data(MqttPublisherDataHandle_t publishDataHandle, struct bme280_data* comp_data)
{
    char valueStr[10];

    snprintf(valueStr, sizeof(valueStr), "%0.2f", comp_data->temperature);
    UpdatePublishData(publishDataHandle, "temperature", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%0.2f", comp_data->pressure / 100.0);
    UpdatePublishData(publishDataHandle, "pressure", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%0.2f", comp_data->humidity);
    UpdatePublishData(publishDataHandle, "humidity", valueStr);
}

void bme280_task(void *arg)
{
    int8_t rslt;
    bme280_task_param_t* param = (bme280_task_param_t*) arg;
    if (param == NULL) {
        ESP_LOGE(TAG, "failed to load task parameter");
        return;
    }
    uint32_t task_idx = param->task_idx;
    //i2c_port_t i2c_port_num = param->i2c_port_num;
    uint8_t settings_sel;

    TickType_t xLastWakeTime;

    struct bme280_dev dev;
    struct bme280_data comp_data;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    ESP_LOGI(TAG, "TASK[%d] initialize device", task_idx);
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "TASK[%d] failed to initialize BME280", task_idx);
        return;
    }

    print_sensor_settings(task_idx, &dev);

    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_1X;
    dev.settings.osr_t = BME280_OVERSAMPLING_1X;
    dev.settings.filter = BME280_FILTER_COEFF_OFF;
    dev.settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    settings_sel = BME280_ALL_SETTINGS_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "TASK[%d] failed to set sensor settings", task_idx);
        return;
    }

    rslt = bme280_get_sensor_settings(&dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "TASK[%d] failed to get sensor settings", task_idx);
        return;
    }
    
    print_sensor_settings(task_idx, &dev);

    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "TASK[%d] failed to set sensor mode", task_idx);
    }

    ESP_LOGI(TAG, "TASK[%d] chip_id = %x", task_idx, dev.chip_id);

    // throw away the first data for stabilizing sensor
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    ESP_LOGI(TAG, "create publish data handle");
    MqttPublisherDataHandle_t publishDataHandle = CreatePublishDataHandle("bme280");

    ESP_LOGI(TAG, "TASK[%d] start loop", task_idx);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt == BME280_OK) {
            print_sensor_data(task_idx, &comp_data);
            publish_sensor_data(publishDataHandle, &comp_data);
        }

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, LOOP_FREQUENCY);
    }

    DeletePublishDataHandle(publishDataHandle);

    vTaskDelete(NULL);
}