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
#include "demo_mqtt.h"
#include <api_hal_watchdog.h> 
#include "api_hal_i2c.h"

#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"



#define PDP_CONTEXT_APN       "cmnet"
#define PDP_CONTEXT_USERNAME  ""
#define PDP_CONTEXT_PASSWD    ""

#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Main Test Task"

#define SECOND_TASK_STACK_SIZE    (2048 * 2)
#define SECOND_TASK_PRIORITY      1
#define SECOND_TASK_NAME          "MQTT Test Task"

#define MQTT_WATCHDOG_INTERVAL 60*2
#define DMA_DEVICE_ID "22100001"

char willMsg[50] = "GPRS 22100001 disconnected!";
uint8_t imei[16] = "";
uint8_t dataPayload[64] = "";


int counter = 0;

static HANDLE mainTaskHandle = NULL;
static HANDLE secondTaskHandle = NULL;

static HANDLE semMqttStart = NULL;
MQTT_Connect_Info_t ci;

typedef enum{
    MQTT_EVENT_CONNECTED = 0,
    MQTT_EVENT_DISCONNECTED ,
    MQTT_EVENT_MAX
}MQTT_Event_ID_t;

typedef struct {
    MQTT_Event_ID_t id;
    MQTT_Client_t* client;
}MQTT_Event_t;

typedef enum{
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTED       ,
    MQTT_STATUS_MAX
}MQTT_Status_t;

MQTT_Status_t mqttStatus = MQTT_STATUS_DISCONNECTED;

uint16_t soundlevelvalue = 0, mV = 0;
ADC_Config_t config = {
      .channel = ADC_CHANNEL_0,
      .samplePeriod = ADC_SAMPLE_PERIOD_100MS
};


bool AttachActivate()
{
    uint8_t status;
    bool ret = Network_GetAttachStatus(&status);
    if(!ret)
    {
        Trace(2,"get attach staus fail");
        return false;
    }
    Trace(2,"attach status:%d",status);
    if(!status)
    {
        ret = Network_StartAttach();
        if(!ret)
        {
            Trace(2,"network attach fail");
            return false;
        }
    }
    else
    {
        ret = Network_GetActiveStatus(&status);
        if(!ret)
        {
            Trace(2,"get activate staus fail");
            return false;
        }
        Trace(2,"activate status:%d",status);
        if(!status)
        {
            Network_PDP_Context_t context = {
                .apn        = PDP_CONTEXT_APN,
                .userName   = PDP_CONTEXT_USERNAME,
                .userPasswd = PDP_CONTEXT_PASSWD
            };
            Network_StartActive(context);
        }
    }
    return true;
}

static void NetworkEventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
            Trace(2,"network register denied");
            break;

        case API_EVENT_ID_NETWORK_REGISTER_NO:
            Trace(2,"network register no");
            break;
        
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"network register success");
            AttachActivate();
            break;

        case API_EVENT_ID_NETWORK_DETACHED:
            Trace(2,"network detached");
            AttachActivate();
            break;
        case API_EVENT_ID_NETWORK_ATTACH_FAILED:
            Trace(2,"network attach failed");
            AttachActivate();
            break;

        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"network attach success");
            AttachActivate();
            break;

        case API_EVENT_ID_NETWORK_DEACTIVED:
            Trace(2,"network deactived");
            AttachActivate();
            break;

        case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
            Trace(2,"network activate failed");
            AttachActivate();
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"network activate success..");
            if(semMqttStart)
                OS_ReleaseSemaphore(semMqttStart);
            break;

        case API_EVENT_ID_SIGNAL_QUALITY:
            Trace(2,"CSQ:%d",pEvent->param1);
            break;

        default:
            break;
    }
}


static void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {    
        case API_EVENT_ID_NO_SIMCARD:
            Trace(2,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;
        case API_EVENT_ID_SIMCARD_DROP:
            Trace(2,"!!SIM CARD%d DROP!!!!",pEvent->param1);
            break;
        case API_EVENT_ID_SYSTEM_READY:
            Trace(2,"system initialize complete");
            break;
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
            Trace(2,"Network Register DENIED");
        case API_EVENT_ID_NETWORK_REGISTER_NO:
            Trace(2,"Network Register NO ");
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"Network Register SUCCESS");
        case API_EVENT_ID_NETWORK_DETACHED:
        case API_EVENT_ID_NETWORK_ATTACH_FAILED:
            Trace(2,"Network Attach FAILED");
        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"Network Attach SUCCESS");
        case API_EVENT_ID_NETWORK_DEACTIVED:
        case API_EVENT_ID_NETWORK_ACTIVATE_FAILED:
            Trace(2,"Network Activate FAILED");
        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"Network Activate SUCCESS");
        case API_EVENT_ID_SIGNAL_QUALITY:
            NetworkEventDispatch(pEvent);
            break;

        default:
            break;
    }
}

// void getSoundLevel()
// {    
//     if(ADC_Read(ADC_CHANNEL_0, &soundlevelvalue, &mV))
//         Trace(1,"ADC value:%d, %dmV",soundlevelvalue,mV);
//     else
//         Trace(1,"ADC not available");

//     OS_Sleep(250);
// }

void measure(){

    int16_t err = 0;

    OS_Sleep(20);    

    // Trace(2, "Data iteration %d", i);
    // Read Measurement

    sensirion_i2c_hal_sleep_usec(1000000);

    uint16_t mass_concentration_pm1p0;
    uint16_t mass_concentration_pm2p5;
    uint16_t mass_concentration_pm4p0;
    uint16_t mass_concentration_pm10p0;
    int16_t ambient_humidity;
    int16_t ambient_temperature; 
    int16_t voc_index;
    int16_t nox_index;

 
    err = sen5x_read_measured_values(
           &mass_concentration_pm1p0, &mass_concentration_pm2p5,
           &mass_concentration_pm4p0, &mass_concentration_pm10p0,
           &ambient_humidity, &ambient_temperature, &voc_index, &nox_index);
    if (err) {
           printf("Error executing sen5x_read_measured_values(): %i\n", err);
           snprintf(dataPayload, sizeof(dataPayload), "%s,ERROR!!,%d", DMA_DEVICE_ID,err);
           printf(dataPayload);
        //    measure();        
       } else {
           printf("Mass concentration pm1p0: %.1f µg/m³\n",
                  mass_concentration_pm1p0 / 10.0f);
           printf("Mass concentration pm2p5: %.1f µg/m³\n",
                  mass_concentration_pm2p5 / 10.0f);
           printf("Mass concentration pm4p0: %.1f µg/m³\n",
                  mass_concentration_pm4p0 / 10.0f);
           printf("Mass concentration pm10p0: %.1f µg/m³\n",
                  mass_concentration_pm10p0 / 10.0f);
           if (ambient_humidity == 0x7fff) {
               printf("Ambient humidity: n/a\n");
           } else {
               printf("Ambient humidity: %.1f percent RH\n",
                      ambient_humidity / 100.0f);
           }
           if (ambient_temperature == 0x7fff) {
               printf("Ambient temperature: n/a\n");
           } else {
               printf("Ambient temperature: %.1f °C\n",
                      ambient_temperature / 200.0f);
           }
           if (voc_index == 0x7fff) {
               printf("Voc index: n/a\n");
           } else {
               printf("Voc index: %.1f\n", voc_index / 10.0f);
           }
           if (nox_index == 0x7fff) {
               printf("Nox index: n/a\n");
           } else {
               printf("Nox index: %.1f\n", nox_index / 10.0f);
           }           
            snprintf(dataPayload, 
                       sizeof(dataPayload), 
                       "%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", 
                       DMA_DEVICE_ID, 
                       mass_concentration_pm1p0 / 10.0f,
                       mass_concentration_pm2p5 / 10.0f,
                       mass_concentration_pm4p0 / 10.0f,
                       mass_concentration_pm10p0 / 10.0f,
                       ambient_humidity / 100.0f,
                       ambient_temperature / 200.0f,
                       voc_index / 10.0f,
                       nox_index / 10.0f
                    );
       }
    
}

void OnMqttConnection(MQTT_Client_t *client, void *arg, MQTT_Connection_Status_t status)
{
    Trace(1,"MQTT connection status:%d",status);
    MQTT_Event_t* event = (MQTT_Event_t*)OS_Malloc(sizeof(MQTT_Event_t));
    if(!event)
    {
        Trace(1,"MQTT no memory");
        return ;
    }
    if(status == MQTT_CONNECTION_ACCEPTED)
    {
        Trace(1,"MQTT succeed connect to broker");
        //!!! DO NOT suscribe here(interrupt function), do MQTT suscribe in task, or it will not excute
        event->id = MQTT_EVENT_CONNECTED;
        event->client = client;
        OS_SendEvent(secondTaskHandle,event,OS_TIME_OUT_WAIT_FOREVER,OS_EVENT_PRI_NORMAL);
    }
    else
    {
        event->id = MQTT_EVENT_DISCONNECTED;
        event->client = client;
        OS_SendEvent(secondTaskHandle,event,OS_TIME_OUT_WAIT_FOREVER,OS_EVENT_PRI_NORMAL);
        Trace(1,"MQTT connect to broker fail,error code:%d",status);
    }
    Trace(1,"MQTT OnMqttConnection() end");
}

static uint32_t reconnectInterval = 3000;
void StartTimerPublish(uint32_t interval,MQTT_Client_t* client);
void StartTimerConnect(uint32_t interval,MQTT_Client_t* client);
void OnPublish(void* arg, MQTT_Error_t err)
{
    if(err == MQTT_ERROR_NONE)
        Trace(1,"MQTT publish success");
    else
        Trace(1,"MQTT publish error, error code:%d",err);
}


void OnTimerPublish(void* param)
{
    MQTT_Error_t err;
    MQTT_Client_t* client = (MQTT_Client_t*)param;
    uint8_t status = MQTT_IsConnected(client);
    Trace(1,"mqtt status:%d",status);


    if(mqttStatus != MQTT_STATUS_CONNECTED)
    {
        Trace(1,"MQTT not connected to broker! can not publish");
        return;
    }

    measure();

    Trace(1,"MQTT OnTimerPublish");
    err = MQTT_Publish(client,PUBLISH_TOPIC,dataPayload,strlen(dataPayload),1,2,0,OnPublish,NULL);
    if(err != MQTT_ERROR_NONE)
        Trace(1,"MQTT publish error, error code:%d",err);
    StartTimerPublish(PUBLISH_INTERVAL,client);
    // counter++; 
}

void StartTimerPublish(uint32_t interval,MQTT_Client_t* client)
{
    OS_StartCallbackTimer(mainTaskHandle,interval,OnTimerPublish,(void*)client);
}

void OnTimerStartConnect(void* param)
{
    MQTT_Error_t err;
    MQTT_Client_t* client = (MQTT_Client_t*)param;
    uint8_t status = MQTT_IsConnected(client);
    Trace(1,"mqtt status:%d",status);
    if(mqttStatus == MQTT_STATUS_CONNECTED)
    {
        Trace(1,"already connected!");
        return ;
    }
    err = MQTT_Connect(client,BROKER_IP,BROKER_PORT,OnMqttConnection,NULL,&ci);
    if(err != MQTT_ERROR_NONE)
    {
        Trace(1,"MQTT connect fail,error code:%d",err);
        reconnectInterval += 1000;
        if(reconnectInterval >= 60000)
            reconnectInterval = 60000;
        StartTimerConnect(reconnectInterval,client);
    }
}

void StartTimerConnect(uint32_t interval,MQTT_Client_t* client)
{
    OS_StartCallbackTimer(mainTaskHandle,interval,OnTimerStartConnect,(void*)client);
}


void SecondTaskEventDispatch(MQTT_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case MQTT_EVENT_CONNECTED:
            reconnectInterval = 3000;
            mqttStatus = MQTT_STATUS_CONNECTED;           
            StartTimerPublish(PUBLISH_INTERVAL,pEvent->client);
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqttStatus = MQTT_STATUS_DISCONNECTED;
            StartTimerConnect(reconnectInterval,pEvent->client);
            break;
        default:
            break;
    }
}


void MQTT_Init(){
    INFO_GetIMEI(imei);
    Trace(1,"IMEI:%s",imei);    

    MQTT_Client_t* client = MQTT_ClientNew();
    
    MQTT_Error_t err;
    memset(&ci,0,sizeof(MQTT_Connect_Info_t));
    ci.client_id = imei;
    ci.client_user = CLIENT_USER;
    ci.client_pass = CLIENT_PASS;
    ci.keep_alive = 20;
    ci.clean_session = 1;
    ci.use_ssl = false;
    ci.will_qos = 2;
    ci.will_topic = "will";
    ci.will_retain = 1;
    memcpy(strstr(willMsg,"GPRS")+5,imei,15);
    ci.will_msg = willMsg;

    err = MQTT_Connect(client,BROKER_IP,BROKER_PORT,OnMqttConnection,NULL,&ci);
    if(err != MQTT_ERROR_NONE)
        Trace(1,"MQTT connect fail,error code:%d",err);
}

void sensorionBegin(){
    int16_t error = 0;

    // sensirion_i2c_hal_init();
       
    OS_Sleep(3000);

    error = sen5x_device_reset();
    if (error) {
        printf("Error executing sen5x_device_reset(): %i\n", error);
        OS_Sleep(1000);
    }else{
        printf("Device reset success !!");
        OS_Sleep(3000);
    }

    unsigned char serial_number[32];
    uint8_t serial_number_size = 32;
    error = sen5x_get_serial_number(serial_number, serial_number_size);
    if (error) {
        printf("Error executing sen5x_get_serial_number(): %i\n", error);
    } else {
        printf("Serial number: %s\n", serial_number);
    }

    unsigned char product_name[32];
    uint8_t product_name_size = 32;
    error = sen5x_get_product_name(product_name, product_name_size);
    if (error) {
        printf("Error executing sen5x_get_product_name(): %i\n", error);
    } else {
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

    if (error) {
        printf("Error executing sen5x_get_version(): %i\n", error);
    } else {
        printf("Firmware: %u.%u, Hardware: %u.%u\n", firmware_major,
               firmware_minor, hardware_major, hardware_minor);
    }

    // uint32_t device_Status[32];
    // uint32_t deviceStatus_size = 32; 
    // error = sen5x_read_device_status(device_Status);
    // if (error) {
    //     printf("Error executing sen5x_read_device_status(): %i\n", error);
    // } else {
    //     Trace(1, device_Status);
    // }

   
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
    float temp_offset = 0.0f;
    int16_t default_slope = 0;
    uint16_t default_time_constant = 0;
    error = sen5x_set_temperature_offset_parameters(
        (int16_t)(200 * temp_offset), default_slope, default_time_constant);
    if (error) {
        printf(
            "Error executing sen5x_set_temperature_offset_parameters(): %i\n",
            error);
    } else {
        printf("Temperature Offset set to %.2f ° C (SEN54/SEN55 only)\n", temp_offset);
    }
    
    
    // uint32_t * deviceStatus = 0;    
    // error =  sen5x_read_device_status(&deviceStatus);
    // if (error) {
    //     printf("Error executing sen5x_read_data_ready(): %i \t data status: %i \n", error, (int)deviceStatus);
    // }
    // else{
    //     printf("sen5x_read_data_ready started successfully\t data status: %i", (int)deviceStatus);
    // }
    
    error = sen5x_start_measurement();
    if (error) {
        printf("Error executing sen5x_start_measurement(): %i\n", error);
    }
    else{
        printf("Measurement started successfully");
    }
}



void SecondTask(void *pData)
{
    MQTT_Event_t* event=NULL;
    
    WatchDog_Open(WATCHDOG_SECOND_TO_TICK(MQTT_WATCHDOG_INTERVAL));
    
    semMqttStart = OS_CreateSemaphore(0);
    OS_WaitForSemaphore(semMqttStart, OS_WAIT_FOREVER);
    OS_DeleteSemaphore(semMqttStart);
    semMqttStart = NULL;

    WatchDog_KeepAlive();


    int16_t error = 0;

    OS_Sleep(10000);

    sensirion_i2c_hal_init();
    OS_Sleep(1000);
    sensorionBegin(); 
    OS_Sleep(1000);
    measure();   

    MQTT_Init();
    
    while(1)
    {
        if(OS_WaitEvent(secondTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            SecondTaskEventDispatch(event);
            OS_Free(event);
        }
    }
}

void MainTask(void *pData)
{
    API_Event_t* event=NULL;

    secondTaskHandle = OS_CreateTask(SecondTask,
        NULL, NULL, SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0, SECOND_TASK_NAME);

    while(1)
    {
        if(OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void afksenmqtt2_Main(void)
{
    mainTaskHandle = OS_CreateTask(MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}