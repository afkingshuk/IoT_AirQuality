#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_hal_i2c.h"

#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"




#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Main Test Task"

#define SECOND_TASK_STACK_SIZE    (2048 * 2)
#define SECOND_TASK_PRIORITY      1
#define SECOND_TASK_NAME          "Second Test Task"

#define I2C_DEFAULT_TIME_OUT_C 25

static HANDLE mainTaskHandle = NULL;
static HANDLE secondTaskHandle = NULL;
#define I2C_ACC I2C2
#define I2C_ADD 0x69

int16_t err = 0;
int16_t error;


void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;

        case API_EVENT_ID_SYSTEM_READY:
            Trace(1,"system initialize complete");
            break;

        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"network register success");
            break;

        default:
            break;
    }
}

void measure(){

//    error = sen5x_device_reset();
//     if (error) {
//         printf("Error executing sen5x_device_reset(): %i\n", error);
//         OS_Sleep(1000);
//     }else{
//         printf("Device reset success 2nd time!!");
//         OS_Sleep(3000);
//     }
//     OS_Sleep(5000);
//     error = sen5x_start_measurement();
//     if (error) {
//         printf("Error executing sen5x_start_measurement(): %i\n", error);
//     }
//     else{
//         printf("Measurement started successfully");
//         OS_Sleep(3000);
//     }

    OS_Sleep(20);

    for (int i = 0; i < 60; i++) {

        Trace(2, "Data iteration %d", i);

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
        }
    }
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
    printf("start of SecondTask ... ");
    Trace(1, "2nd task ");

    int16_t error = 0;

    OS_Sleep(10000);

    sensirion_i2c_hal_init();
    OS_Sleep(1000);
    sensorionBegin(); 
    OS_Sleep(1000);
    // sensirion_i2c_hal_free();
    // OS_Sleep(1000);

    // int c=0;
    // while(c < 10){
    //     I2C_Error_t err = I2C_Close(I2C2);
    //     if(err){
    //          Trace(1, "%d : I2C Close failed.  code: %d", c, err);
    //     } else {
    //         Trace(1, "I2C Close freed ... successful !!");
    //         break;
    //     }
    //  OS_Sleep(2000);
    //  c++; 
    // }

    // OS_Sleep(1000);

    
    measure();    

       
}



void MainTask(void *pData)
{
    Trace(1, "V2.1.1");
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

void afksenm3_Main(void)
{
    mainTaskHandle = OS_CreateTask(MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}