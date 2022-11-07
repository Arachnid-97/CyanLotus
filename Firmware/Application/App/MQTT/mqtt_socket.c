#include "mqtt_socket.h"
#include <string.h>
#include "MQTTFreeRTOS.h"
#include "MQTTClient.h"

#include "netconf.h"

#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "MQTTPacket.h"


extern SemaphoreHandle_t MuxSem_UartPrintf;
#define MQTT_PRINTF(fmt,arg...)         do{\
                                            xSemaphoreTake(MuxSem_UartPrintf, portMAX_DELAY);\
                                            printf("<<-MQTT->> %s > "fmt"", __FUNCTION__, ##arg);\
                                            xSemaphoreGive(MuxSem_UartPrintf);\
                                        }while(0)


/* connect to m2m.eclipse.org, subscribe to a topic, send and receive messages regularly every 1 sec */
MQTTClient MQTT_client;
Network MQTT_network;
unsigned char sendbuf[80], readbuf[80];


static void messageArrived(MessageData* data)
{
	MQTT_PRINTF("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
		data->message->payloadlen, (char *)data->message->payload);
}

static int publishData(MQTTClient *client, const char *topic, char *payload)
{
    MQTTMessage message;
    int rc = 0;

    message.qos = QOS1;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    if ((rc = MQTTPublish(client, topic, &message)) != 0)
        MQTT_PRINTF("Return code from MQTT publish is %d\n", rc);

    return rc;
}

void MQTT_Link(MQTTClient *client)
{
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	int rc = 0;

    connectData.clientID.cstring = CLIENT_ID;              //随机
    connectData.username.cstring = USER_NAME;              //用户名
    connectData.password.cstring = PASSWORD;               //秘钥
    connectData.MQTTVersion = MQTT_VERSION;                //3表示3.1版本，4表示3.11版本
    connectData.keepAliveInterval = KEEPLIVE_TIME;         //保持活跃
    connectData.cleansession = 1;

    while(1)
    {
        do
        {
            if (!netif_is_link_up(&xnetif))
            { /* waiting for link up */
                rc = NetworkConnect(client->ipstack, MQTT_HOST_NAME, MQTT_HOST_PORT);
            }
            vTaskDelay(3000 / portTICK_RATE_MS);
        } while (rc < 0);

        rc = MQTTConnect(client, &connectData);
        if(rc != 0){
            client->ipstack->disconnect(client->ipstack);
            MQTT_PRINTF("Return code from MQTT connect is %d\n", rc);
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }
        else
            MQTT_PRINTF("MQTT Connected\n");

        rc = MQTTSubscribe(client, TOPIC, QOS1, messageArrived);
        if(rc != 0)
        {
            client->ipstack->disconnect(client->ipstack);
            MQTT_PRINTF("Return code from MQTT subscribe is %d\n", rc);
            continue;
        }
        else
            break;
    }
}

void MQTT_UnLink(MQTTClient *client)
{
    MQTTDisconnect(client);
    client->ipstack->disconnect(client->ipstack);
}

int MQTT_Push(MQTTClient *client)
{
    char payload[30];
    int rc = -1;

    strcpy(payload, TEST_MESSAGE);

    rc = publishData(client, TOPIC, payload);

    return rc;
}

#if 0
int MQTT_Poll(void *pvParameters)
{
    int rc;
    Timer timer;
    MQTTClient* c = (MQTTClient*)pvParameters;

    TimerInit(&timer);

    do
    {
#if defined(MQTT_TASK)
        MutexLock(&c->mutex);
#endif
        TimerCountdownMS(&timer, 500); /* Don't wait too long if no traffic is incoming */
        rc = cycle(c, &timer);
#if defined(MQTT_TASK)
        MutexUnlock(&c->mutex);
#endif

        if (rc < 0)
            return FAILURE;
    } while (!TimerIsExpired(&timer));

    return SUCCESS;
}
#endif

void vMQTT_Task(void *pvParameters)
{
    unsigned char* sendbuf = NULL;
    unsigned char* readbuf = NULL;
    volatile int ret = 0;

    // 分配缓存
    sendbuf = (unsigned char *)mem_malloc(MQTT_TXBUF_SIZE);
    if (sendbuf < 0)
        MQTT_PRINTF("MQTT failed to mem_malloc sendbuf!\r\n");
    readbuf = (unsigned char *)mem_malloc(MQTT_RXBUF_SIZE);
    if (readbuf < 0)
        MQTT_PRINTF("MQTT failed to mem_malloc readbuf!\r\n");

    NetworkInit(&MQTT_network);
    MQTTClientInit(&MQTT_client, &MQTT_network, 30000, sendbuf, (size_t)MQTT_TXBUF_SIZE, readbuf, (size_t)MQTT_RXBUF_SIZE);
    MQTT_Link(&MQTT_client);

#if defined(MQTT_TASK)
    int rc;
    if ((rc = MQTTStartTask(&MQTT_client)) != pdPASS)
        MQTT_PRINTF("Return code from start tasks is %d\n", rc);
#endif

    while (1)
    {

#if !defined(MQTT_TASK)
		if ((ret = MQTTYield(&MQTT_client, 1000)) != 0)
			MQTT_PRINTF("Return code from yield is %d\n", ret);
#endif

        vTaskDelay(3000/portTICK_RATE_MS);
        if((ret = MQTT_Push(&MQTT_client)) == 0)
            continue;

        MQTT_UnLink(&MQTT_client);

        MQTT_Link(&MQTT_client);
    }

    // mem_free(sendbuf);
    // mem_free(readbuf);

    // vTaskDelete(NULL);
}

void MQTT_Init(void)
{
    sys_thread_new("vMQTT_Task", vMQTT_Task, NULL, 1024, DEFAULT_THREAD_PRIO);
}
