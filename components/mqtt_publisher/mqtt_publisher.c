#include <string.h>
#include <search.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* AWS System includes. */
#include "aws_clientcredential.h"
#include "aws_mqtt_agent.h"

#include "esp_log.h"

/* local header */
#include "mqtt_publisher.h"


static const TickType_t LOOP_FREQUENCY = 60000 / portTICK_RATE_MS;
static const TickType_t MAX_MQTT_TIMEOUT = 1000 / portTICK_RATE_MS;
static const TickType_t PUBLISH_DATA_HANDLE_LIST_MUTEX_TIMEOUT = 1000 / portTICK_RATE_MS;
static const char *TAG = "MQTT Publisher";

#define PUBLISH_MAX_TOPIC_SIZE (128)
#define PUBLISH_MAX_MESSAGE_SIZE (512)
#define PUBLISH_DATA_HANDLE_LIST_MAX_SIZE (16)

static MqttPublisherDataHandle_t publishDataHandleList[PUBLISH_DATA_HANDLE_LIST_MAX_SIZE];

static SemaphoreHandle_t publishDataHandleListMutex;

static const char publish_Topic_Path[] = "iot_env_sensor/%s/sensor";


MqttPublisherError_t prvInitializeMqttClient(MQTTAgentHandle_t* mqttClientHandle)
{
    MQTTAgentConnectParams_t connectParams;

    if (MQTT_AGENT_Create(mqttClientHandle) != eMQTTAgentSuccess) {
        ESP_LOGI(TAG, "failed to create MQTT Client");
        return -1;
    }

    // initialize connection parameters
    connectParams.pucClientId = (const uint8_t*) clientcredentialIOT_THING_NAME;
    connectParams.usClientIdLength = (uint16_t) strlen(clientcredentialIOT_THING_NAME);
    connectParams.pcURL = clientcredentialMQTT_BROKER_ENDPOINT;
    connectParams.usPort = clientcredentialMQTT_BROKER_PORT;
    connectParams.xFlags = mqttagentREQUIRE_TLS | mqttagentUSE_AWS_IOT_ALPN_443;    
    connectParams.xURLIsIPAddress = pdFALSE;    /* Deprecated. */
    connectParams.xSecuredConnection = pdFALSE; /* Deprecated. */
    connectParams.pcCertificate = NULL;
    connectParams.ulCertificateSize = 0;
    connectParams.pvUserData = NULL;
    connectParams.pxCallback = NULL;

    ESP_LOGI(TAG, "connecting to %s:%d", connectParams.pcURL, connectParams.usPort);

    TickType_t delay = 2000 / portTICK_RATE_MS;

    for (int i = 0; i < 5; i++) {
        MQTTAgentReturnCode_t retCode = MQTT_AGENT_Connect(*mqttClientHandle, &connectParams, MAX_MQTT_TIMEOUT);
        if (retCode == eMQTTAgentSuccess) {
            return MQTT_PUBLISHER_OK;
        }
        vTaskDelay(delay);
        delay = delay * 2;
    }

    ESP_LOGE(TAG, "failed to connect MQTT Client");
    return MQTT_PUBLISHER_ERROR;
}

MqttPublisherError_t prvCreatePublishMessage(const MqttPublisherDataHandle_t handle, char* message, size_t messageSize)
{
    ESP_LOGI(TAG, "creating publish message, handle name = %s", handle->name);

    int lastMessageSize = (int) messageSize;
    lastMessageSize -= 2; // for '{' and '}'
    if (lastMessageSize < 0) {
        ESP_LOGW(TAG, "message buffer size is too short, size = %d, handle name = %s", (int) messageSize, handle->name);
        return MQTT_PUBLISHER_ERROR;
    }

    strcat(message, "{");

    for (int i = 0; i < handle->pairArraySize; i++) {
        lastMessageSize -= (5 + strlen(handle->pairArray[i]->key) + strlen(handle->pairArray[i]->value));
        if (lastMessageSize < 0) {
            ESP_LOGW(TAG, "message buffer size is too short, size = %d, handle name = %s", (int) messageSize, handle->name);
            return MQTT_PUBLISHER_ERROR;
        }
        strcat(message, "\"");
        strcat(message, handle->pairArray[i]->key);
        strcat(message, "\":\"");
        strcat(message, handle->pairArray[i]->value);
        strcat(message, "\"");

        if (i != handle->pairArraySize - 1) {
            lastMessageSize--;
            if (lastMessageSize < 0) {
                ESP_LOGW(TAG, "message buffer size is too short, size = %d, handle name = %s", (int) messageSize, handle->name);
                return MQTT_PUBLISHER_ERROR;
            }
            strcat(message, ",");
        }
    }

    strcat(message, "}");

    return MQTT_PUBLISHER_OK;
}

void prvPublishSensorData(MQTTAgentHandle_t mqttClientHandle, MQTTQoS_t mqttQos, const char* topic, const char* message)
{
    ESP_LOGI(TAG, "publish message, topic = %s, message = %s", topic, message);

    MQTTAgentPublishParams_t publishParams;

    memset(&publishParams, 0x00, sizeof(publishParams));
    publishParams.xQoS = mqttQos;
    publishParams.pucTopic = (const uint8_t*) topic;
    publishParams.usTopicLength = (uint16_t) strlen(topic);
    publishParams.pvData = message;
    publishParams.ulDataLength = strlen(message);
    
    if (MQTT_AGENT_Publish(mqttClientHandle, &publishParams, MAX_MQTT_TIMEOUT) != eMQTTAgentSuccess)
    {
        ESP_LOGW(TAG, "failed to publish MQTT message, topic = %s, message = %s", topic, message);
    }
}

void prvPublishHandleList(MQTTAgentHandle_t mqttClientHandle)
{
    char topic[PUBLISH_MAX_TOPIC_SIZE] = "";
    char totalMessage[PUBLISH_MAX_MESSAGE_SIZE] = "";
    char message[PUBLISH_MAX_MESSAGE_SIZE] = "";

    if (xSemaphoreTake(publishDataHandleListMutex, PUBLISH_DATA_HANDLE_LIST_MUTEX_TIMEOUT) == pdFALSE) {
        ESP_LOGW(TAG, "failed to take mutex");
        return;
    }

    if (snprintf(topic, PUBLISH_MAX_TOPIC_SIZE, publish_Topic_Path, clientcredentialIOT_THING_NAME) < 0) {
        // error
    }

    int lastTotalMessage = PUBLISH_MAX_MESSAGE_SIZE;
    lastTotalMessage -= 2; // for '{' and '}'  
    if (lastTotalMessage < 0) {
        ESP_LOGW(TAG, "message buffer size is too short");
        return;
    }

    strcat(totalMessage, "{");
    bool isFirst = true;
    for (int i = 0; i < PUBLISH_DATA_HANDLE_LIST_MAX_SIZE; i++) {
        if (publishDataHandleList[i]) {
            message[0] = '\0';
            if (prvCreatePublishMessage(publishDataHandleList[i], message, PUBLISH_MAX_MESSAGE_SIZE) != MQTT_PUBLISHER_OK) {
                ESP_LOGW(TAG, "failed to create publish message, handle name = %s", publishDataHandleList[i]->name);
                break;
            }
            if (!isFirst) {
                lastTotalMessage--;
                if (lastTotalMessage < 0) {
                    ESP_LOGW(TAG, "message buffer size is too short");
                    return;
                }
                strcat(totalMessage, ",");
            }
            isFirst = false;

            lastTotalMessage -= (3 + strlen(publishDataHandleList[i]->name) + strlen(message));
            if (lastTotalMessage < 0) {
                ESP_LOGW(TAG, "message buffer size is too short");
                return;
            }

            strcat(totalMessage, "\"");
            strcat(totalMessage, publishDataHandleList[i]->name);
            strcat(totalMessage, "\":");
            strcat(totalMessage, message);
        }
    }
    strcat(totalMessage, "}");

    prvPublishSensorData(mqttClientHandle, eMQTTQoS0, topic, totalMessage);

    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }
}

void prvMqttPublishTask(void *arg)
{
    publishDataHandleListMutex = xSemaphoreCreateMutex();

    MQTTAgentHandle_t mqttClientHandle;
    if (prvInitializeMqttClient(&mqttClientHandle) != MQTT_PUBLISHER_OK) {
        ESP_LOGI(TAG, "failed to initialize MQTT Client");
        return;
    }

    ESP_LOGI(TAG, "start loop");

    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        ESP_LOGI(TAG, "start proc");

        prvPublishHandleList(mqttClientHandle);

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, LOOP_FREQUENCY);
    }

    MQTT_AGENT_Delete(mqttClientHandle);

    vSemaphoreDelete(publishDataHandleListMutex);

    vTaskDelete(NULL);
}

MqttPublisherKeyValuePair* prvCreateKeyValuePair()
{
    return (MqttPublisherKeyValuePair*) pvPortMalloc(sizeof(MqttPublisherKeyValuePair));
}

void prvDeleteKeyValuePair(MqttPublisherDataHandle_t handle)
{
    for (size_t i = 0; i < handle->pairArraySize; i++) {
        vPortFree(handle->pairArray[i]);
        handle->pairArray[i] = NULL;
    }
    handle->pairArraySize = 0;
}

/**
 * find KeyValuPair specified by argument
 * 
 * @param[in] array retrieval object
 * @param[in] size size of array
 * @param[in] key 
 */
MqttPublisherKeyValuePair* prvFindKeyValuePair(MqttPublisherDataHandle_t handle, const char* key)
{
    for (size_t i = 0; i < handle->pairArraySize; i++) {
        if (strcmp(handle->pairArray[i]->key, key) == 0) {
            return handle->pairArray[i];
        }
    }
    return NULL;
}

MqttPublisherError_t prvAddKeyValuePair(MqttPublisherDataHandle_t handle, MqttPublisherKeyValuePair* pair)
{
    if (handle->pairArraySize >= (sizeof(handle->pairArray) / sizeof(MqttPublisherKeyValuePair*))) {
        ESP_LOGW(TAG, "no enough space of array, handle name = %s, key = %s, value = %s", handle->name, pair->key, pair->value);
        return MQTT_PUBLISHER_ERROR;
    }

    handle->pairArray[handle->pairArraySize] = pair;
    handle->pairArraySize++;
    return MQTT_PUBLISHER_OK;
}

MqttPublisherDataHandle_t CreatePublishDataHandle(char* name)
{
    ESP_LOGI(TAG, "create publish data handle, name = %s", name);

    MqttPublisherDataHandle_t handle = (MqttPublisherDataHandle_t) pvPortMalloc(sizeof(MqttPublisherData));

    memset(handle, 0, sizeof(*handle));

    strncpy(handle->name, name, sizeof(handle->name) / sizeof(char));

    if (xSemaphoreTake(publishDataHandleListMutex, PUBLISH_DATA_HANDLE_LIST_MUTEX_TIMEOUT) == pdFALSE) {
        ESP_LOGW(TAG, "failed to take mutex");
        vPortFree(handle);
        goto error;
    }

    // add handle to PublishDatahandleList
    for (int i = 0; i < PUBLISH_DATA_HANDLE_LIST_MAX_SIZE; i++) {
        if (publishDataHandleList[i] == NULL) {
            publishDataHandleList[i] = handle;
            break;
        }
    }

    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }

    return handle;

error:
    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }
    return NULL;
}

MqttPublisherError_t UpdatePublishData(MqttPublisherDataHandle_t handle, const char* key, const char* value)
{
    ESP_LOGI(TAG, "update publish data, key = %s, value = %s", key, value);

    if (handle == NULL) {
        ESP_LOGW(TAG, "invalid handle");
        return MQTT_PUBLISHER_ERROR;
    }
    
    int keySize = strnlen(key, MQTT_PUBLISHER_KEY_TEXT_SIZE);
    if (keySize >= MQTT_PUBLISHER_KEY_TEXT_SIZE) {
        ESP_LOGW(TAG, "failed to update publish data ,invalid key size, size = %d", keySize);
        return MQTT_PUBLISHER_ERROR;
    }

    int valueSize = strnlen(key, MQTT_PUBLISHER_VALUE_TEXT_SIZE);
    if (valueSize >= MQTT_PUBLISHER_VALUE_TEXT_SIZE) {
        ESP_LOGW(TAG, "failed to update publish data ,invalid value size, size = %d", valueSize);
        return MQTT_PUBLISHER_ERROR;
    }

    if (xSemaphoreTake(publishDataHandleListMutex, PUBLISH_DATA_HANDLE_LIST_MUTEX_TIMEOUT) == pdFALSE) {
        ESP_LOGW(TAG, "failed to take mutex");
        return MQTT_PUBLISHER_ERROR;
    }

    MqttPublisherKeyValuePair* pair = prvFindKeyValuePair(handle, key);
    if (pair == NULL) {
        ESP_LOGI(TAG, "create new key value pair");
        pair = prvCreateKeyValuePair();
        strncpy(pair->key, key, MQTT_PUBLISHER_KEY_TEXT_SIZE);

        if (prvAddKeyValuePair(handle, pair) != MQTT_PUBLISHER_OK) {
            ESP_LOGW(TAG, "failed to add key value pair");
            goto error;
        }
    }

    strncpy(pair->value, value, MQTT_PUBLISHER_VALUE_TEXT_SIZE);

    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }
    return MQTT_PUBLISHER_OK;

error:
    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }
    return MQTT_PUBLISHER_ERROR;
}

void DeletePublishDataHandle(MqttPublisherDataHandle_t handle)
{
    ESP_LOGI(TAG, "delete publish data handle, name = %s", handle->name);

    if (handle == NULL) {
        ESP_LOGW(TAG, "invalid handle");
        return;
    }
    
    if (xSemaphoreTake(publishDataHandleListMutex, PUBLISH_DATA_HANDLE_LIST_MUTEX_TIMEOUT) == pdFALSE) {
        ESP_LOGW(TAG, "failed to take mutex");
        return;    
    }

    // delete handle from publishDataHandleList
    for (int i = 0; i < PUBLISH_DATA_HANDLE_LIST_MAX_SIZE; i++) {
        if (publishDataHandleList[i] == handle) {
            publishDataHandleList[i] = NULL;
        }
    }

    if (xSemaphoreGive(publishDataHandleListMutex) == pdFALSE) {
        ESP_LOGW(TAG, "failed to give mutex");
    }

    prvDeleteKeyValuePair(handle);
    vPortFree(handle);
}

void StartMqttPublisherTask()
{
    xTaskCreate(prvMqttPublishTask, "MqttPublishTask", 1024 * 5, NULL, 10, NULL);
}