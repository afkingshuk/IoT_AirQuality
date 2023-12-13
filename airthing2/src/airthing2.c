#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_mqtt.h"
#include "api_network.h"
#include "api_socket.h"
#include "api_info.h"
#include "mqtt.h"
// #include "pm_task.h"
#include "api_hal_gpio.h"
#include "api_hal_watchdog.h"

// #include "config.h"

#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include "api_hal_pm.h"
#include "time.h"
#include "api_info.h"
#include "assert.h"
#include "api_socket.h"
#include "api_network.h"
#include "api_lbs.h"
#include "api_sms.h"
#include "api_hal_uart.h"
#include "ntp.h"

// Sensor headers
#include "api_hal_i2c.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

// #define NTP_SERVER "cn.ntp.org.cn"
#define NTP_SERVER "pool.ntp.org"

#define PDP_CONTEXT_APN "cmnet"
#define PDP_CONTEXT_USERNAME ""
#define PDP_CONTEXT_PASSWD ""

#define MAIN_TASK_STACK_SIZE (2048 * 2)
#define MAIN_TASK_PRIORITY 0
#define MAIN_TASK_NAME "Main Test Task"

#define MQTT_TASK_STACK_SIZE (2048 * 2)
#define MQTT_TASK_PRIORITY 1
#define MQTT_TASK_NAME "MQTT Test Task"

#define SMS_TASK_STACK_SIZE (2048 * 2)
#define SMS_TASK_PRIORITY 1
#define SMS_TASK_NAME "MQTT Test Task"

#define GPS_TASK_STACK_SIZE (2048 * 2)
#define GPS_TASK_PRIORITY 0
#define GPS_TASK_NAME "GPS Task"

#define SENSOR_TASK_STACK_SIZE (2048 * 2)
#define SENSOR_TASK_PRIORITY 0
#define SENSOR_TASK_NAME "Sensor Task"

#define GPS_NMEA_LOG_FILE_PATH "/t/gps_nmea.log"

// Config
#define DUID "C49"
// ON OFF RESTART
#define ON "ON"
#define OFF "OFF"
#define SLEEP "SLEEP"
#define RESTART "RESTART"

#define GPSPin GPIO_PIN27
#define StatePin GPIO_PIN28
#define IgnitionPin GPIO_PIN29

char willMsg[50] = "GPRS 123456789012345 disconnected!";
uint8_t imei[16] = "";
char pubTopic[64] = "";
char subTopic[64] = "";
char CSQ[8] = "0";
uint8_t percent;

time_t timeNTP = 0;
time_t timeNow;

static MQTT_Client_t *mqttClient;

static HANDLE mainTaskHandle = NULL;
static HANDLE mqttTaskHandle = NULL;
static HANDLE gpsTaskHandle = NULL;
static HANDLE sensorTaskHandle = NULL;
static HANDLE smsTaskHandle = NULL;

static uint8_t flag = 0; // sms

bool isGpsOn = true;
bool networkFlag = false;
HANDLE semGetCellInfo = NULL;

float latitudeLbs = 0.0;
float longitudeLbs = 0.0;

char payload[512] = "kingshuk";

bool isLowPowerOn = false;

// sensor
#define I2C_DEFAULT_TIME_OUT_C 25
#define SEN5X_I2C_ADDRESS 0x69
#define I2C_ACC I2C2
#define I2C_ADD 0x69
int16_t error = 0;
uint16_t mass_concentration_pm1p0;
uint16_t mass_concentration_pm2p5;
uint16_t mass_concentration_pm4p0;
uint16_t mass_concentration_pm10p0;
int16_t ambient_humidity;
int16_t ambient_temperature;
int16_t voc_index;
int16_t nox_index;

static HANDLE semMqttStart = NULL;
MQTT_Connect_Info_t ci;

typedef enum
{
    MQTT_EVENT_CONNECTED = 0,
    MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_MAX
} MQTT_Event_ID_t;

typedef struct
{
    MQTT_Event_ID_t id;
    MQTT_Client_t *client;
} MQTT_Event_t;

typedef enum
{
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_MAX
} MQTT_Status_t;

MQTT_Status_t mqttStatus = MQTT_STATUS_DISCONNECTED;

GPIO_config_t stateLed = {
    .mode = GPIO_MODE_OUTPUT,
    .pin = StatePin,
    .defaultLevel = GPIO_LEVEL_LOW};
GPIO_config_t gpsLed = {
    .mode = GPIO_MODE_OUTPUT,
    .pin = GPSPin,
    .defaultLevel = GPIO_LEVEL_LOW};
GPIO_config_t ingLed = {
    .mode = GPIO_MODE_OUTPUT,
    .pin = IgnitionPin,
    .defaultLevel = GPIO_LEVEL_HIGH};

bool AttachActivate()
{
    uint8_t status;
    bool ret = Network_GetAttachStatus(&status);
    if (!ret)
    {
        Trace(2, "get attach staus fail");
        return false;
    }
    Trace(2, "attach status:%d", status);
    if (!status)
    {
        ret = Network_StartAttach();
        if (!ret)
        {
            Trace(2, "network attach fail");
            return false;
        }
    }
    else
    {
        ret = Network_GetActiveStatus(&status);
        if (!ret)
        {
            Trace(2, "get activate staus fail");
            return false;
        }
        Trace(2, "activate status:%d", status);
        if (!status)
        {
            Network_PDP_Context_t context = {
                .apn = PDP_CONTEXT_APN,
                .userName = PDP_CONTEXT_USERNAME,
                .userPasswd = PDP_CONTEXT_PASSWD};
            Network_StartActive(context);
        }
    }
    return true;
}

static void NetworkEventDispatch(API_Event_t *pEvent)
{
    switch (pEvent->id)
    {
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
        Trace(2, "network register denied");
        break;

    case API_EVENT_ID_NETWORK_REGISTER_NO:
        Trace(2, "network register no");
        break;

    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
        Trace(2, "network register success");
        AttachActivate();
        break;

    case API_EVENT_ID_NETWORK_DETACHED:
        Trace(2, "network detached");
        AttachActivate();
        break;
    case API_EVENT_ID_NETWORK_ATTACH_FAILED:
        Trace(2, "network attach failed");
        AttachActivate();
        break;

    case API_EVENT_ID_NETWORK_ATTACHED:
        Trace(2, "network attach success");
        AttachActivate();
        break;

    case API_EVENT_ID_NETWORK_DEACTIVED:
        Trace(2, "network deactived");
        AttachActivate();
        break;

    case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
        Trace(2, "network activate failed");
        AttachActivate();
        break;

    case API_EVENT_ID_NETWORK_ACTIVATED:
        Trace(2, "network activate success..");
        networkFlag = true;
        if (semMqttStart)
            OS_ReleaseSemaphore(semMqttStart);
        break;
    case API_EVENT_ID_NETWORK_GOT_TIME:
        Trace(2, "network got time");
        break;

    case API_EVENT_ID_SIGNAL_QUALITY:
        Trace(2, "CSQ:%d", pEvent->param1);
        snprintf(CSQ, sizeof(CSQ), "%d", pEvent->param1);
        break;

    default:
        break;
    }
}
// void GPSPublish(pParam1)
// {
//     Trace(1, "received GPS data,length:%d, data:%s ", pParam1, pParam1);
// }

static void EventDispatch(API_Event_t *pEvent)
{
    static uint8_t lbsCount = 0;
    switch (pEvent->id)
    {
    case API_EVENT_ID_NO_SIMCARD:
        Trace(2, "!!NO SIM CARD%d!!!!", pEvent->param1);
        break;
    case API_EVENT_ID_SIMCARD_DROP:
        Trace(2, "!!SIM CARD%d DROP!!!!", pEvent->param1);
        break;
    case API_EVENT_ID_SYSTEM_READY:
        Trace(2, "system initialize complete");
        break;
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
    case API_EVENT_ID_NETWORK_REGISTER_NO:
    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
    case API_EVENT_ID_NETWORK_DETACHED:
    case API_EVENT_ID_NETWORK_ATTACH_FAILED:
    case API_EVENT_ID_NETWORK_ATTACHED:
    case API_EVENT_ID_NETWORK_DEACTIVED:
    case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
    case API_EVENT_ID_NETWORK_ACTIVATED:
    case API_EVENT_ID_SIGNAL_QUALITY:
        NetworkEventDispatch(pEvent);
        break;

    case API_EVENT_ID_GPS_UART_RECEIVED:
        // GPSPublish(pEvent->pParam1);
        // Trace(1, "received GPS data,length:%d, data:%s,flag:%d", pEvent->param1, pEvent->pParam1, flag);
        GPS_Update(pEvent->pParam1, pEvent->param1);
        break;

    case API_EVENT_ID_UART_RECEIVED:
        if (pEvent->param1 == UART1)
        {
            uint8_t data[pEvent->param2 + 1];
            data[pEvent->param2] = 0;
            memcpy(data, pEvent->pParam1, pEvent->param2);
            Trace(1, "uart received data,length:%d,data:%s", pEvent->param2, data);
            if (strcmp(data, "close") == 0)
            {
                Trace(1, "close gps");
                GPS_Close();
                isGpsOn = false;
            }
            else if (strcmp(data, "open") == 0)
            {
                Trace(1, "open gps");
                GPS_Open(NULL);
                isGpsOn = true;
            }
        }
        break;
    case API_EVENT_ID_NETWORK_CELL_INFO:
    {
        // uint8_t number = pEvent->param1;
        // Network_Location_t *location = (Network_Location_t *)pEvent->pParam1;
        // Trace(2, "network cell infomation,serving cell number:1, neighbor cell number:%d", number - 1);

        // for (int i = 0; i < number; ++i)
        // {
        //     Trace(2, "cell %d info:%d%d%d,%d%d%d,%d,%d,%d,%d,%d,%d", i,
        //           location[i].sMcc[0], location[i].sMcc[1], location[i].sMcc[2],
        //           location[i].sMnc[0], location[i].sMnc[1], location[i].sMnc[2],
        //           location[i].sLac, location[i].sCellID, location[i].iBsic,
        //           location[i].iRxLev, location[i].iRxLevSub, location[i].nArfcn);
        // }

        // if (!LBS_GetLocation(location, number, 15, &longitudeLbs, &latitudeLbs))
        //     Trace(1, "===LBS get location fail===");
        // else
        //     Trace(1, "===LBS get location success, latitude:%f,longitude:%f===", latitudeLbs, longitudeLbs);
        // if ((latitudeLbs == 0) && (longitudeLbs == 0)) // not get location from server, try again
        // {
        //     if (++lbsCount > 3)
        //     {
        //         lbsCount = 0;
        //         Trace(1, "try 3 times to get location from lbs but fail!!");
        //         OS_ReleaseSemaphore(semGetCellInfo);
        //         break;
        //     }
        //     if (!Network_GetCellInfoRequst())
        //     {
        //         Trace(1, "network get cell info fail");
        //         OS_ReleaseSemaphore(semGetCellInfo);
        //     }
        //     break;
        // }
        // OS_ReleaseSemaphore(semGetCellInfo);
        // lbsCount = 0;
        break;
    }
    case API_EVENT_ID_SMS_SENT:
        Trace(2, "Send Message Success");
        break;
    case API_EVENT_ID_SMS_RECEIVED:
        Trace(6, "received message");
        SMS_Encode_Type_t encodeType = pEvent->param1;
        uint32_t contentLength = pEvent->param2;
        uint8_t *header = pEvent->pParam1;
        uint8_t *content = pEvent->pParam2;

        Trace(6, "message header:%s", header);
        Trace(6, "message content length:%d", contentLength);
        if (encodeType == SMS_ENCODE_TYPE_ASCII)
        {
            Trace(6, "message content:%s", content);
            UART_Write(UART1, content, contentLength);
            if (strcmp(content, ON) == 0)
            {
                Trace(6, "SMS Command: %s", content);
                GPIO_SetLevel(ingLed, GPIO_LEVEL_HIGH);
                isLowPowerOn = false;
                PM_SleepMode(false);
            }
            if (strcmp(content, OFF) == 0)
            {
                Trace(6, "SMS Command: %s", content);
                GPIO_SetLevel(ingLed, GPIO_LEVEL_LOW);
                PM_ShutDown();
            }
            if (strcmp(content, SLEEP) == 0)
            {
                Trace(6, "SMS Command: %s", content);
                isLowPowerOn = true;
                PM_SleepMode(true);
                GPIO_SetLevel(ingLed, GPIO_LEVEL_LOW);
            }
            if (strcmp(content, RESTART) == 0)
            {
                Trace(6, "SMS Command: %s", content);
                PM_Reboot();
            }
        }
        else
        {
            uint8_t tmp[500];
            memset(tmp, 0, 500);
            for (int i = 0; i < contentLength; i += 2)
                sprintf(tmp + strlen(tmp), "\\u%02x%02x", content[i], content[i + 1]);
            Trace(2, "message content(unicode):%s", tmp); // you can copy this string to http://tool.chinaz.com/tools/unicode.aspx and display as Chinese
            uint8_t *gbk = NULL;
            uint32_t gbkLen = 0;
            if (!SMS_Unicode2LocalLanguage(content, contentLength, CHARSET_CP936, &gbk, &gbkLen))
                Trace(10, "convert unicode to GBK fail!");
            else
            {
                memset(tmp, 0, 500);
                for (int i = 0; i < gbkLen; i += 2)
                    sprintf(tmp + strlen(tmp), "%02x%02x ", gbk[i], gbk[i + 1]);
                Trace(2, "message content(GBK):%s", tmp); // you can copy this string to http://m.3158bbs.com/tool-54.html# and display as Chinese
                UART_Write(UART1, gbk, gbkLen);           // use serial tool that support GBK decode if have Chinese, eg: https://github.com/Neutree/COMTool
            }
            OS_Free(gbk);
        }
        break;
    case API_EVENT_ID_SMS_LIST_MESSAGE:
    {
        SMS_Message_Info_t *messageInfo = (SMS_Message_Info_t *)pEvent->pParam1;
        Trace(1, "message header index:%d,status:%d,number type:%d,number:%s,time:\"%u/%02u/%02u,%02u:%02u:%02u+%02d\"", messageInfo->index, messageInfo->status,
              messageInfo->phoneNumberType, messageInfo->phoneNumber,
              messageInfo->time.year, messageInfo->time.month, messageInfo->time.day,
              messageInfo->time.hour, messageInfo->time.minute, messageInfo->time.second,
              messageInfo->time.timeZone);
        Trace(1, "message content len:%d,data:%s", messageInfo->dataLen, messageInfo->data);
        UART_Write(UART1, messageInfo->data, messageInfo->dataLen); // use serial tool that support GBK decode if have Chinese, eg: https://github.com/Neutree/COMTool
        UART_Write(UART1, "\r\n\r\n", 4);
        // need to free data here
        OS_Free(messageInfo->data);
        break;
    }
    case API_EVENT_ID_SMS_ERROR:
        Trace(10, "SMS error occured! cause:%d", pEvent->param1);

    default:
        break;
    }
}

void OnMqttReceived(void *arg, const char *topic, uint32_t payloadLen)
{
    Trace(1, "MQTT received publish data request, topic:%s, payload length:%d", topic, payloadLen);
}

void OnMqttReceiedData(void *arg, const uint8_t *data, uint16_t len, MQTT_Flags_t flags)
{
    char rData[255];
    memset(rData, '\0', sizeof(rData));
    strncpy(rData, data, len);

    Trace(1, "MQTT recieved data , length:%d,data:%s", len, rData);
    // switch ON OFF RESTART
    if (strcmp(rData, ON) == 0)
    {
        Trace(1, "Command: ON!!");
        GPIO_SetLevel(ingLed, GPIO_LEVEL_HIGH);
        isLowPowerOn = false;
        PM_SleepMode(false);
    }
    if (strcmp(rData, OFF) == 0)
    {
        Trace(1, "Command: OFF!!");
        GPIO_SetLevel(ingLed, GPIO_LEVEL_LOW);
        PM_ShutDown();
    }
    if (strcmp(rData, SLEEP) == 0)
    {
        Trace(1, "Command: SLEEP!!");
        isLowPowerOn = true;
        PM_SleepMode(true);
        GPIO_SetLevel(ingLed, GPIO_LEVEL_LOW);
    }
    if (strcmp(rData, RESTART) == 0)
    {
        Trace(1, "Command: RESTART!!");
        Trace(2, "Restarting ... ");

        PM_Reboot();
    }

    if (flags == MQTT_FLAG_DATA_LAST)
        Trace(1, "MQTT data is last frame");
}

void OnMqttSubscribed(void *arg, MQTT_Error_t err)
{
    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT subscribe fail, error code:%d", err);
    else
        Trace(1, "MQTT subscribe success, topic:%s", (const char *)arg);
}

void OnMqttConnection(MQTT_Client_t *client, void *arg, MQTT_Connection_Status_t status)
{
    Trace(1, "MQTT connection status:%d", status);
    MQTT_Event_t *event = (MQTT_Event_t *)OS_Malloc(sizeof(MQTT_Event_t));
    if (!event)
    {
        Trace(1, "MQTT no memory");
        return;
    }
    if (status == MQTT_CONNECTION_ACCEPTED)
    {
        Trace(1, "MQTT succeed connect to broker");
        //!!! DO NOT suscribe here(interrupt function), do MQTT suscribe in task, or it will not excute
        event->id = MQTT_EVENT_CONNECTED;
        event->client = client;
        OS_SendEvent(mqttTaskHandle, event, OS_TIME_OUT_WAIT_FOREVER, OS_EVENT_PRI_NORMAL);
    }
    else
    {
        event->id = MQTT_EVENT_DISCONNECTED;
        event->client = client;
        OS_SendEvent(mqttTaskHandle, event, OS_TIME_OUT_WAIT_FOREVER, OS_EVENT_PRI_NORMAL);
        Trace(1, "MQTT connect to broker fail,error code:%d", status);
    }
    Trace(1, "MQTT OnMqttConnection() end");
}

static uint32_t reconnectInterval = 3000;
void StartTimerPublish(uint32_t interval, MQTT_Client_t *client);
void StartTimerConnect(uint32_t interval, MQTT_Client_t *client);
void OnPublish(void *arg, MQTT_Error_t err)
{
    if (err == MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT publish success");
    }
    else
        Trace(1, "MQTT publish error, error code:%d", err);
}

void OnTimerPublish(void *param)
{
    MQTT_Error_t err;
    MQTT_Client_t *client = (MQTT_Client_t *)param;
    uint8_t status = MQTT_IsConnected(client);
    Trace(1, "mqtt status:%d", status);
    if (mqttStatus != MQTT_STATUS_CONNECTED)
    {
        Trace(1, "MQTT not connected to broker! can not publish");
        return;
    }

    Trace(1, "MQTT OnTimerPublish");

    // err = MQTT_Publish(client, pubTopic, payload, strlen(payload), 1, 2, 0, OnPublish, NULL);
    // GPIO_SetLevel(stateLed, GPIO_LEVEL_LOW);
    // if (err != MQTT_ERROR_NONE)
    //     Trace(1, "MQTT publish error, error code:%d", err);
    StartTimerPublish(PUBLISH_INTERVAL, client);
}

void StartTimerPublish(uint32_t interval, MQTT_Client_t *client)
{
    OS_StartCallbackTimer(mainTaskHandle, interval, OnTimerPublish, (void *)client);
}

void OnTimerStartConnect(void *param)
{
    MQTT_Error_t err;
    MQTT_Client_t *client = (MQTT_Client_t *)param;
    uint8_t status = MQTT_IsConnected(client);
    Trace(1, "mqtt status:%d", status);
    if (mqttStatus == MQTT_STATUS_CONNECTED)
    {
        Trace(1, "already connected!");
        return;
    }
    err = MQTT_Connect(client, BROKER_IP, BROKER_PORT, OnMqttConnection, NULL, &ci);
    if (err != MQTT_ERROR_NONE)
    {
        Trace(1, "MQTT connect fail,error code:%d", err);
        reconnectInterval += 1000;
        if (reconnectInterval >= 60000)
            reconnectInterval = 60000;
        StartTimerConnect(reconnectInterval, client);
    }
}

void StartTimerConnect(uint32_t interval, MQTT_Client_t *client)
{
    OS_StartCallbackTimer(mainTaskHandle, interval, OnTimerStartConnect, (void *)client);
}

void MQTTTaskEventDispatch(MQTT_Event_t *pEvent)
{
    switch (pEvent->id)
    {
    case MQTT_EVENT_CONNECTED:
        reconnectInterval = 3000;
        mqttStatus = MQTT_STATUS_CONNECTED;
        Trace(1, "MQTT connected, now subscribe topic:%s", subTopic);
        MQTT_Error_t err;
        mqttClient = pEvent->client;
        MQTT_SetInPubCallback(pEvent->client, OnMqttReceived, OnMqttReceiedData, NULL);
        err = MQTT_Subscribe(pEvent->client, subTopic, 2, OnMqttSubscribed, (void *)SUBSCRIBE_TOPIC);
        if (err != MQTT_ERROR_NONE)
            Trace(1, "MQTT subscribe error, error code:%d", err);
        StartTimerPublish(PUBLISH_INTERVAL, pEvent->client);
        break;
    case MQTT_EVENT_DISCONNECTED:
        mqttStatus = MQTT_STATUS_DISCONNECTED;
        StartTimerConnect(reconnectInterval, pEvent->client);
        break;
    default:
        break;
    }
}

// void MQTT_GPS_Publish(void *param, buffer)
// {
//     MQTT_Error_t err;
//     MQTT_Client_t *client = (MQTT_Client_t *)param;
//     uint8_t status = MQTT_IsConnected(client);
//     Trace(1, "mqtt status:%d", status);
//     if (mqttStatus != MQTT_STATUS_CONNECTED)
//     {
//         Trace(1, "MQTT not connected to broker! can not publish");
//         return;
//     }
//     Trace(1, "MQTT OnTimerPublish");

//     // Publish messages to the MQTT server (broker)
//     //  client: MQTT client object
//     //  topic: topic
//     //  port: server port
//     //  payload: message body
//     //  payloadLen: message body length
//     //  dup: Indicates the number of repetitions sent
//     //  qos: quality of service
//     //  retain: The server needs to persist the message
//     //  callback: post request callback function
//     //  arg: the parameters that need to be passed to the callback function

//     err = MQTT_Publish(client, pubTopic, postDate, strlen(postDate), 1, 2, 0, OnPublish, NULL);

//     if (err != MQTT_ERROR_NONE)
//         Trace(1, "MQTT publish error, error code:%d", err);
// }

// void sensorionBegin()
// {
//     unsigned char serial_number[32];
//     uint8_t serial_number_size = 32;
//     error = sen5x_get_serial_number(serial_number, serial_number_size);
//     if (error)
//     {
//         printf("Error executing sen5x_get_serial_number(): %i\n", error);
//     }
//     else
//     {
//         printf("Serial number: %s\n", serial_number);
//     }

//     unsigned char product_name[32];
//     uint8_t product_name_size = 32;
//     error = sen5x_get_product_name(product_name, product_name_size);
//     if (error)
//     {
//         printf("Error executing sen5x_get_product_name(): %i\n", error);
//     }
//     else
//     {
//         printf("Product name: %s\n", product_name);
//     }

//     uint8_t firmware_major;
//     uint8_t firmware_minor;
//     bool firmware_debug;
//     uint8_t hardware_major;
//     uint8_t hardware_minor;
//     uint8_t protocol_major;
//     uint8_t protocol_minor;
//     error = sen5x_get_version(&firmware_major, &firmware_minor, &firmware_debug,
//                               &hardware_major, &hardware_minor, &protocol_major,
//                               &protocol_minor);

//     if (error)
//     {
//         printf("Error executing sen5x_get_version(): %i\n", error);
//     }
//     else
//     {
//         printf("Firmware: %u.%u, Hardware: %u.%u\n", firmware_major,
//                firmware_minor, hardware_major, hardware_minor);
//     }
// }

void gps_testTask(void *pData)
{
    GPS_Info_t *gpsInfo = Gps_GetInfo();
    uint8_t buffer[1022];
    // wait for gprs register complete
    // The process of GPRS registration network may cause the power supply voltage of GPS to drop,
    // which resulting in GPS restart.

    // open GPS hardware(UART2 open either)
    GPS_Init();
    GPS_SaveLog(true, GPS_NMEA_LOG_FILE_PATH);
    // if(!GPS_ClearLog())
    //     Trace(1,"open file error, please check tf card");
    GPS_Open(NULL);

    while (!networkFlag)
    {
        Trace(1, "wait for gprs regiter complete");
        OS_Sleep(2000);
    }
    // Watchdog check
    WatchDog_KeepAlive();

    if (networkFlag)
    {

        for (int i = 0; i < 3; i++)
        {
            if (timeNTP <= 0)
            {
                if (NTP_Update(NTP_SERVER, 5, &timeNTP, true) != 0)
                    timeNTP = 0;
                if (timeNTP > 0)
                {
                    Trace(1, "ntp get time success,time:%u", timeNTP);
                    Trace(1, "timestamp:%d Time: %s", timeNTP, ctime((const time_t *)&timeNTP));
                }
            }
            timeNow = time(NULL);
            // Trace(1, "time rtc now:%s", ctime((const time_t *)&timeNow));
            Trace(1, "time rtc now:%d", timeNow);
            OS_Sleep(10);
        }
    }

    // wait for gps start up, or gps will not response command
    while (gpsInfo->rmc.latitude.value == 0)
        OS_Sleep(1000);

    // set gps nmea output interval as 10s
    for (uint8_t i = 0; i < 5; ++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(1, "set gps ret:%d", ret);
        if (ret)
            break;
        OS_Sleep(500);
    }
    // get version of gps firmware
    if (!GPS_GetVersion(buffer, 150))
        Trace(1, "get gps firmware version fail");
    else
        Trace(1, "gps firmware version:%s", buffer);

    // get location through LBS
    // semGetCellInfo = OS_CreateSemaphore(0);
    // if (!Network_GetCellInfoRequst())
    // {
    //     Trace(1, "network get cell info fail");
    // }
    // OS_WaitForSemaphore(semGetCellInfo, OS_TIME_OUT_WAIT_FOREVER);
    // OS_DeleteSemaphore(semGetCellInfo);
    // semGetCellInfo = NULL;

    // send location to GPS and update brdc GPD file
    Trace(1, "do AGPS now");
    if (!GPS_AGPS(latitudeLbs, longitudeLbs, 0, true))
    {
        Trace(1, "agps fail");
    }
    else
    {
        Trace(1, "do AGPS success");
    }

    // set nmea output interval as 1s
    if (!GPS_SetOutputInterval(1000))
        Trace(1, "set nmea output interval fail");

    Trace(1, "init ok");

    while (1)
    {
        WatchDog_KeepAlive();
        GPIO_SetLevel(gpsLed, GPIO_LEVEL_HIGH);
        int counter = 0;
        double latSum = 0.0, longSum = 0.0;
        while (counter < 5)
        {

            if (isGpsOn)
            {

                // show fix info
                uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ? gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;
                char *isFixedStr = NULL;
                if (isFixed == 2)
                    isFixedStr = "2";
                else if (isFixed == 3)
                {
                    if (gpsInfo->gga.fix_quality == 1)
                        isFixedStr = "3";
                    else if (gpsInfo->gga.fix_quality == 2)
                        isFixedStr = "4";
                }
                else
                    isFixedStr = "0";

                if (isFixedStr != 0)
                {
                    GPIO_SetLevel(gpsLed, GPIO_LEVEL_LOW);
                }
                else
                {
                    GPIO_SetLevel(gpsLed, GPIO_LEVEL_HIGH);
                }

                // convert unit ddmm.mmmm to degree(°)
                int temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
                double latitude = temp + (double)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
                temp = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
                double longitude = temp + (double)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) / gpsInfo->rmc.longitude.scale / 60.0;

                latSum += latitude;
                longSum += longitude;
                // you can copy ` latitude,longitude ` to https://www.google.com/maps check location on map
                if (!isLowPowerOn)
                {
                    if (gpsInfo->gga.fix_quality > 0)
                    {
                        GPIO_SetLevel(gpsLed, GPIO_LEVEL_HIGH);
                        OS_Sleep(150);
                        GPIO_SetLevel(gpsLed, GPIO_LEVEL_LOW);
                    }
                }

                // Trace(4, "VTg:%s ", (float)gpsInfo->vtg);
                float speedN = 0.0, speedK = 0.0, headT = 0.0, headR = 0.0;

                switch (gpsInfo->vtg.faa_mode)
                {
                case 'N':
                    // Trace(4, "FAA:%c MINMEA_FAA_MODE_NOT_VALID", gpsInfo->vtg.faa_mode);
                    // Trace(4, "Relative Head:%.1f ", NAN);
                    // Trace(4, "True Head:%.1f", NAN);
                    // Trace(4, "Speed:%.1f Knots", NAN);
                    // Trace(4, "SPEED:%.1f Kmph", NAN);
                    speedN = NAN;
                    speedK = NAN;
                    headT = NAN;
                    headR = NAN;
                    break;
                case 'A':
                    speedK = (float)gpsInfo->vtg.true_track_degrees.value / 100;
                    headT = (float)gpsInfo->vtg.speed_knots.value / 1000;
                    // Trace(4, "FAA:%c MINMEA_FAA_MODE_AUTONOMOUS", gpsInfo->vtg.faa_mode);
                    // Trace(4, "Relative Head:%.1f ", headR);
                    // Trace(4, "True Head:%.1f", headT);
                    // Trace(4, "Speed:%.1f Knots", speedN);
                    // Trace(4, "SPEED:%.1f Kmph", speedK);

                    break;
                case 'D':
                    speedK = (float)gpsInfo->vtg.true_track_degrees.value / 100;
                    headT = (float)gpsInfo->vtg.speed_knots.value / 1000;
                    // Trace(4, "FAA:%c MINMEA_FAA_MODE_DIFFERENTIAL", gpsInfo->vtg.faa_mode);
                    // Trace(4, "Relative Head:%.1f ", headR);
                    // Trace(4, "True Head:%.1f", headT);
                    // Trace(4, "Speed:%.1f Knots", speedN);
                    // Trace(4, "SPEED:%.1f Kmph", speedK);
                    break;

                    // case 'P':
                    //     speedN = (float)gpsInfo->vtg.magnetic_track_degrees.value / 100;
                    //     speedK = (float)gpsInfo->vtg.true_track_degrees.value / 100;
                    //     headT = (float)gpsInfo->vtg.speed_knots.value / 1000;
                    //     headR = (float)gpsInfo->vtg.speed_kph.value / 1000;
                    Trace(4, "FAA:%c MINMEA_FAA_MODE_PRECISE", gpsInfo->vtg.faa_mode);
                    Trace(4, "True Head:%.1f", headT);
                    Trace(4, "SPEED:%.1f Kmph", speedK);
                    break;

                default:
                    break;
                }

                uint16_t v = PM_Voltage(&percent);
                // Trace(1, "power:%d %d", v, percent);

                snprintf(buffer, sizeof(buffer),
                         "%s,%d,%d,%d,%d/%d,%s,%f,%f,%f,%s,%d, %.1f,%.1f,%.1f,%.1f",
                         DUID,
                         gpsInfo->gsa[0].fix_type,
                         gpsInfo->gsa[1].fix_type,
                         gpsInfo->gga.fix_quality,
                         gpsInfo->gga.satellites_tracked,
                         gpsInfo->gsv[0].total_sats,
                         isFixedStr,
                         latitude,
                         longitude,
                         gpsInfo->gga.altitude,
                         CSQ,
                         percent,
                         (float)gpsInfo->vtg.magnetic_track_degrees.value / 100,
                         (float)gpsInfo->vtg.true_track_degrees.value / 100,
                         (float)gpsInfo->vtg.speed_knots.value / 1000,
                         (float)gpsInfo->vtg.speed_kph.value / 1000);
                // show in tracer
                Trace(1, buffer);
                // payload = buffer;
                // snprintf(payload, sizeof(payload), buffer);

                // send to UART1
                UART_Write(UART1, buffer, strlen(buffer));
                UART_Write(UART1, "\r\n\r\n", 4);

                // // send to server
                // char *requestPath = buffer2;
                // uint8_t percent;
                // uint16_t v = PM_Voltage(&percent);
                // Trace(1, "power:%d %d", v, percent);
                // memset(buffer, 0, sizeof(buffer));
                // if (!INFO_GetIMEI(buffer))
                //     Assert(false, "NO IMEI");
                // Trace(1, "device name:%s", buffer);
                // snprintf(requestPath, sizeof(buffer2), "/?id=%s&timestamp=%d&lat=%f&lon=%f&speed=%f&bearing=%.1f&altitude=%f&accuracy=%.1f&batt=%.1f",
                //          buffer, time(NULL), latitude, longitude, isFixed * 1.0, 0.0, gpsInfo->gga.altitude, 0.0, percent * 1.0);

                // // publish

                // GPIO_SetLevel(stateLed, GPIO_LEVEL_HIGH);
                // Trace(1, "MQTT GPS publish try");
                // MQTT_Error_t err = MQTT_Publish(mqttClient, pubTopic, buffer, strlen(buffer), 1, 2, 0, OnPublish, NULL);
                // if (err != MQTT_ERROR_NONE)
                //     Trace(1, "MQTT publish error, error code:%d", err);
                // // StartTimerPublish(PUBLISH_INTERVAL, client);
                // Trace(1, "MQTT GPS publish success");
                // GPIO_SetLevel(stateLed, GPIO_LEVEL_LOW);

                // if (counter > 9)
                // {

                //
            }

            OS_Sleep(500);

            counter++;
        }
        latSum = latSum / counter;
        longSum = longSum / counter;
        Trace(3, "Average: %f, %f", latSum, longSum);
        timeNow = time(NULL);
        // Trace(1, "time rtc now:%s", ctime((const time_t *)&timeNow));
        snprintf(buffer, sizeof(buffer),
                 "%s,%d,%d,%d/%d,%f,%f,%s,%d,%.1f,%.1f ",
                 DUID,
                 timeNow,
                 gpsInfo->gga.fix_quality,
                 gpsInfo->gga.satellites_tracked,
                 gpsInfo->gsv[0].total_sats,
                 latSum,
                 longSum,
                 CSQ,
                 percent,
                 (float)gpsInfo->vtg.true_track_degrees.value / 100,
                 (float)gpsInfo->vtg.speed_kph.value / 1000);
        // publish
        if (!isLowPowerOn)
        {
            GPIO_SetLevel(stateLed, GPIO_LEVEL_HIGH);
        }
        if (mqttStatus)
        {

            Trace(1, "MQTT GPS publish try");
            MQTT_Error_t err = MQTT_Publish(mqttClient, pubTopic, buffer, strlen(buffer), 1, 2, 0, OnPublish, NULL);
            if (err != MQTT_ERROR_NONE)
                Trace(1, "MQTT publish error, error code:%d", err);
            WatchDog_KeepAlive();
            Trace(1, "MQTT GPS publish success");
            GPIO_SetLevel(stateLed, GPIO_LEVEL_LOW);
        }
        counter = 0;
        // }

        OS_Sleep(1000);
    }
}

void sensor_testTask(void *pData)
{

    // sensirion_i2c_hal_init();

    error = sen5x_device_reset();
    if (error)
    {
        printf("Error executing sen5x_device_reset(): %i\n", error);
        OS_Sleep(1500);
        PM_Restart();
    }
    else
    {
        printf("Device reset success!");
    }

    // set a temperature offset - supported by SEN54 and SEN55 sensors
    //
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used in
    // `sen5x_set_temperature_offset_parameters`,
    // `sen5x_set_warm_start_parameter` and
    // `sen5x_set_rht_acceleration_mode`.
    //
    // Adjust temp_offset in degrees celsius to account for additional
    // temperature offsets exceeding the SEN module's self heating.

    // float temp_offset = 0.0f;
    // int16_t default_slope = 0;
    // uint16_t default_time_constant = 0;

    // error = sen5x_set_temperature_offset_parameters(
    //     (int16_t)(200 * temp_offset), default_slope, default_time_constant);
    // if (error) {
    //     printf(
    //         "Error executing sen5x_set_temperature_offset_parameters(): %i\n", error);
    // } else {
    //     printf("Temperature Offset set to %.2f °C (SEN54/SEN55 only)\n", temp_offset);
    // }

    // Start Measurement
    // error = sen5x_start_measurement();
    // if (error) {
    //     printf("Error executing sen5x_start_measurement(): %i\n", error);
    // }

    /**
     * @brief eikhan thekhe ghotona ghotar kotha thakleo beshir bhag somoy error 14 return ashtese :'(
     * immediate code block function e kora ase but test er jonno eikhane lekha ... function block comment kora.
     *
     *
     */

    // int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x21);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error)
    {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(50000);

    while (1)
    {
        OS_Sleep(5000);
        // Read Measurement
        sensirion_i2c_hal_sleep_usec(1000000);

        // uint16_t mass_concentration_pm1p0;
        // uint16_t mass_concentration_pm2p5;
        // uint16_t mass_concentration_pm4p0;
        // uint16_t mass_concentration_pm10p0;
        // int16_t ambient_humidity;
        // int16_t ambient_temperature;
        // int16_t voc_index;
        // int16_t nox_index;

        error = sen5x_read_measured_values(
            &mass_concentration_pm1p0, &mass_concentration_pm2p5,
            &mass_concentration_pm4p0, &mass_concentration_pm10p0,
            &ambient_humidity, &ambient_temperature, &voc_index, &nox_index);

        if (error)
        {
            printf("Error executing sen5x_read_measured_values(): %i\n", error);
            OS_Sleep(1500);
            // PM_Restart();
        }
        else
        {
            printf("Mass concentration pm1p0: %.1f µg/m³\n",
                   mass_concentration_pm1p0 / 10.0f);
            printf("Mass concentration pm2p5: %.1f µg/m³\n",
                   mass_concentration_pm2p5 / 10.0f);
            printf("Mass concentration pm4p0: %.1f µg/m³\n",
                   mass_concentration_pm4p0 / 10.0f);
            printf("Mass concentration pm10p0: %.1f µg/m³\n",
                   mass_concentration_pm10p0 / 10.0f);
            if (ambient_humidity == 0x7fff)
            {
                printf("Ambient humidity: n/a\n");
            }
            else
            {
                printf("Ambient humidity: %.1f %%RH\n",
                       ambient_humidity / 100.0f);
            }
            if (ambient_temperature == 0x7fff)
            {
                printf("Ambient temperature: n/a\n");
            }
            else
            {
                printf("Ambient temperature: %.1f °C\n",
                       ambient_temperature / 200.0f);
            }
            if (voc_index == 0x7fff)
            {
                printf("Voc index: n/a\n");
            }
            else
            {
                printf("Voc index: %.1f\n", voc_index / 10.0f);
            }
            if (nox_index == 0x7fff)
            {
                printf("Nox index: n/a\n");
            }
            else
            {
                printf("Nox index: %.1f\n", nox_index / 10.0f);
            }
        }
        // publish
        if (!isLowPowerOn)
        {
            GPIO_SetLevel(stateLed, GPIO_LEVEL_HIGH);
        }
        if (mqttStatus)
        {

            Trace(1, "MQTT GPS publish try");
            MQTT_Error_t err = MQTT_Publish(mqttClient, pubTopic, buffer, strlen(buffer), 1, 2, 0, OnPublish, NULL);
            if (err != MQTT_ERROR_NONE)
                Trace(1, "MQTT publish error, error code:%d", err);
            WatchDog_KeepAlive();
            Trace(1, "MQTT GPS publish success");
            GPIO_SetLevel(stateLed, GPIO_LEVEL_LOW);
        }
        error = sen5x_stop_measurement();
        if (error)
        {
            printf("Error executing sen5x_stop_measurement(): %i\n", error);
        }
    }

    OS_Sleep(1000);
}

void MQTTTask(void *pData)
{
    WatchDog_Open(WATCHDOG_SECOND_TO_TICK(60 * 2));
    MQTT_Event_t *event = NULL;

    semMqttStart = OS_CreateSemaphore(0);
    OS_WaitForSemaphore(semMqttStart, OS_WAIT_FOREVER);
    OS_DeleteSemaphore(semMqttStart);
    semMqttStart = NULL;

    WatchDog_KeepAlive();

    Trace(1, "start mqtt test");

    INFO_GetIMEI(imei);
    Trace(1, "IMEI:%s", imei);

    snprintf(pubTopic, sizeof(pubTopic), PUBLISH_TOPIC, DUID);
    snprintf(subTopic, sizeof(subTopic), SUBSCRIBE_TOPIC, DUID);
    Trace(1, "SubTopic:%s,  UID:%s", subTopic, DUID);
    Trace(1, "PubTopic:%s, UID:%s", pubTopic, DUID);

    MQTT_Client_t *client = MQTT_ClientNew();

    MQTT_Error_t err;
    memset(&ci, 0, sizeof(MQTT_Connect_Info_t));
    ci.client_id = imei;
    ci.client_user = CLIENT_USER;
    ci.client_pass = CLIENT_PASS;
    ci.keep_alive = 20;
    ci.clean_session = 1;
    ci.use_ssl = false;
    ci.will_qos = 2;
    ci.will_topic = "will";
    ci.will_retain = 1;
    memcpy(strstr(willMsg, "GPRS") + 5, imei, 15);
    ci.will_msg = willMsg;

    err = MQTT_Connect(client, BROKER_IP, BROKER_PORT, OnMqttConnection, NULL, &ci);
    if (err != MQTT_ERROR_NONE)
        Trace(1, "MQTT connect fail,error code:%d", err);

    while (1)
    {
        if (OS_WaitEvent(mqttTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            MQTTTaskEventDispatch(event);
            OS_Free(event);
        }
    }
}
void GPSTask(void *pData)
{
    API_Event_t *event = NULL;

    TIME_SetIsAutoUpdateRtcTime(true);

    // open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent = true};
    UART_Init(UART1, config);

    // //Create UART1 send task and location print task
    OS_CreateTask(gps_testTask,
                  NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);

    // Wait event
    while (1)
    {
        if (OS_WaitEvent(gpsTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}
void SensorTask(void *pData)
{
    API_Event_t *event = NULL;

    TIME_SetIsAutoUpdateRtcTime(true);

    // open I2c2
    I2C_Config_t config;
    config.freq = I2C_FREQ_100K;
    I2C_Init(I2C_ACC, config);

    // sensorionBegin();
    unsigned char serial_number[32];
    uint8_t serial_number_size = 32;
    error = sen5x_get_serial_number(serial_number, serial_number_size);
    if (error)
    {
        printf("Error executing sen5x_get_serial_number(): %i\n", error);
    }
    else
    {
        printf("Serial number: %s\n", serial_number);
    }

    unsigned char product_name[32];
    uint8_t product_name_size = 32;
    error = sen5x_get_product_name(product_name, product_name_size);
    if (error)
    {
        printf("Error executing sen5x_get_product_name(): %i\n", error);
    }
    else
    {
        printf("Product name: %s\n", product_name);
    }

    uint8_t firmware_major;
    uint8_t firmware_minor;
    bool firmware_debug;
    uint8_t hardware_major;
    uint8_t hardware_minor;
    uint8_t protocol_major;
    uint8_t protocol_minor;
    error = sen5x_get_version(&firmware_major, &firmware_minor, &firmware_debug,
                              &hardware_major, &hardware_minor, &protocol_major,
                              &protocol_minor);

    if (error)
    {
        printf("Error executing sen5x_get_version(): %i\n", error);
    }
    else
    {
        printf("Firmware: %u.%u, Hardware: %u.%u\n", firmware_major,
               firmware_minor, hardware_major, hardware_minor);
    }
    // //Create UART1 send task and location print task
    OS_CreateTask(sensor_testTask,
                  NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);

    // Wait event
    while (1)
    {
        if (OS_WaitEvent(gpsTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}
// void SMSInit()
// {
//     if(!SMS_SetFormat(SMS_FORMAT_TEXT,SIM0))
//     {
//         Trace(1,"sms set format error");
//         return;
//     }
//     SMS_Parameter_t smsParam = {
//         .fo = 17 ,
//         .vp = 167,
//         .pid= 0  ,
//         .dcs= 8  ,//0:English 7bit, 4:English 8 bit, 8:Unicode 2 Bytes
//     };
//     if(!SMS_SetParameter(&smsParam,SIM0))
//     {
//         Trace(1,"sms set parameter error");
//         return;
//     }
//     if(!SMS_SetNewMessageStorage(SMS_STORAGE_SIM_CARD))
//     {
//         Trace(1,"sms set message storage fail");
//         return;
//     }
// }

void SMSTask(void *pData)
{
    API_Event_t *event = NULL;

    // uart init
    //  prolly will not be needed
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .rxCallback = NULL,
    };
    UART_Init(UART1, config);

    if (!SMS_SetFormat(SMS_FORMAT_TEXT, SIM0))
    {
        Trace(1, "sms set format error");
        return;
    }
    SMS_Parameter_t smsParam = {
        .fo = 17,
        .vp = 167,
        .pid = 0,
        .dcs = 8, // 0:English 7bit, 4:English 8 bit, 8:Unicode 2 Bytes
    };
    if (!SMS_SetParameter(&smsParam, SIM0))
    {
        Trace(1, "sms set parameter error");
        return;
    }
    if (!SMS_SetNewMessageStorage(SMS_STORAGE_SIM_CARD))
    {
        Trace(1, "sms set message storage fail");
        return;
    }

    while (1)
    {
        if (OS_WaitEvent(smsTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void MainTask(void *pData)
{
    API_Event_t *event = NULL;
    GPIO_Init(stateLed);
    GPIO_Init(ingLed);

    mqttTaskHandle = OS_CreateTask(MQTTTask, NULL, NULL, MQTT_TASK_STACK_SIZE, MQTT_TASK_PRIORITY, 0, 0, MQTT_TASK_NAME);

    gpsTaskHandle = OS_CreateTask(GPSTask, NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);
    sensorTaskHandle = OS_CreateTask(SensorTask, NULL, NULL, SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIORITY, 0, 0, SENSOR_TASK_NAME);
    smsTaskHandle = OS_CreateTask(SMSTask, NULL, NULL, SMS_TASK_STACK_SIZE, SMS_TASK_PRIORITY, 0, 0, SMS_TASK_NAME);

    while (1)
    {
        if (OS_WaitEvent(mainTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void airthing2_Main(void)
{
    mainTaskHandle = OS_CreateTask(MainTask,
                                   NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}