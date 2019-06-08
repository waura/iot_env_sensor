#ifndef _MQTT_PUBLISHER_H_
#define _MQTT_PUBLISHER_H_

#define MQTT_PUBLISHER_KEY_TEXT_SIZE (64)
#define MQTT_PUBLISHER_VALUE_TEXT_SIZE (64)

#define MQTT_PUBLISHER_DATA_NAME_SIZE (64)
#define MQTT_PUBLISHER_NODE_PAIR_ARRAY_SIZE (16)

typedef int MqttPublisherError_t;

enum {
  MQTT_PUBLISHER_OK = 0,
  MQTT_PUBLISHER_ERROR = -1,
};

typedef struct {
    char key[MQTT_PUBLISHER_KEY_TEXT_SIZE];
    char value[MQTT_PUBLISHER_VALUE_TEXT_SIZE];
} MqttPublisherKeyValuePair;

typedef struct {
  char name[MQTT_PUBLISHER_DATA_NAME_SIZE];
  unsigned char pairArraySize;
  MqttPublisherKeyValuePair* pairArray[MQTT_PUBLISHER_NODE_PAIR_ARRAY_SIZE];
} MqttPublisherData;

typedef MqttPublisherData* MqttPublisherDataHandle_t;

extern MqttPublisherDataHandle_t CreatePublishDataHandle(char* name);
extern MqttPublisherError_t UpdatePublishData(MqttPublisherDataHandle_t handle, const char* key, const char* value);
extern void DeletePublishDataHandle(MqttPublisherDataHandle_t handle);

extern void StartMqttPublisherTask();

#endif //_MQTT_PUBLISHER_H_