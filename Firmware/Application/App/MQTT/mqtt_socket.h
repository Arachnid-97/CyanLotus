#ifndef __MQTT_SOCKET_H
#define __MQTT_SOCKET_H


// #include "MQTTClient.h"


#if LWIP_DNS
#define MQTT_HOST_NAME      "iot.eclipse.org"   // 服务器域名
#else
#define MQTT_HOST_NAME      "198.41.30.254"     // 服务器 IP地址
#endif

#define MQTT_HOST_PORT      1883                // 由于是 TCP连接，端口必须是 1883

#define MSG_MAX_LEN         500
#define MSG_TOPIC_LEN       50
#define KEEPLIVE_TIME       50
#define MQTT_VERSION        4

#define CLIENT_ID           "FreeRTOS_sample"   // 客户端 ID
#define USER_NAME           " "                 // 用户名
#define PASSWORD            " "                 // 秘钥

#define TOPIC               "FreeRTOS/sample/#" // 订阅的主题
#define TEST_MESSAGE        "test_message"      // 测试消息

#define MQTT_TXBUF_SIZE     80
#define MQTT_RXBUF_SIZE     1024


// void MQTT_Link(MQTTClient *client);
// void MQTT_UnLink(MQTTClient *client);
// int MQTT_Push(MQTTClient *client);
int MQTT_Poll(void *pvParameters);
void vMQTT_Task(void *pvParameters);
void MQTT_Init(void);


#endif /* __MQTT_SOCKET_H */

