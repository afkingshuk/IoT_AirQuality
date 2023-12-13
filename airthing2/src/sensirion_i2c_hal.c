#include "api_hal_i2c.h"
#include "api_os.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"

#define I2C_ACC I2C2
#define I2C_ADD 0x69

#define SENSIRION_I2C_DEFAULT_TIMEOUT 200


/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */
/**
 * INSTRUCTIONS by Kingshuk
 * =========================
 * implementation must be according to the platform, for this context A9G CSDK.
 * According to their documentation!! there are 8 functions in total.
 * * * 1 init : bool I2C_Init(I2C_ID_t i2c, I2C_Config_t config);
 * * * 3 write: 
 * * * * * * I2C_Error_t I2C_Transmit(I2C_ID_t i2c, uint16_t slaveAddr, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * * * * * * I2C_Error_t I2C_WriteMem(I2C_ID_t i2c, uint16_t slaveAddr, uint32_t memAddr, uint8_t memSize, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * * * * * * I2C_Error_t I2C_WriteRawByte(I2C_ID_t i2c, uint8_t sendByte, I2C_CMD_Mask_t cmdMask, uint32_t timeOut);
 * * * 3 Read:
 * * * * * * I2C_Error_t I2C_Receive(I2C_ID_t i2c, uint16_t slaveAddr, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * * * * * * I2C_Error_t I2C_ReadMem(I2C_ID_t i2c, uint16_t slaveAddr, uint32_t memAddr, uint8_t memSize, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * * * * * * uint8_t I2C_ReadRawByte(I2C_ID_t i2c, I2C_CMD_Mask_t cmdMask, uint32_t timeOut);
 * * * 1 close: bool I2C_Close(I2C_ID_t i2c)
 * 
 * Some definations are mentioned in comment blocks just before it might be needed. 
 * 
 * theoritically and practically only THIS file (functions gula ar ki) need to be changed. all other files are according to the datasheet 
 * including CRC and bit,byte, word etc manipulations. 
 * 
 * I tried to cover all from datasheet and implement and failed to compile successfully. :)
 * there are some implementations provided by Sensirion. I keept em in the supportAssets folder.
 * I found really useful and close to A9G the Nordic_nrf5.  
 * 
 * Dhonnobad. 
 */

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    /* TODO:IMPLEMENT or leave empty if all sensors are located on one single
     * bus
     */
    return NOT_IMPLEMENTED_ERROR;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void) {
    I2C_Config_t config;
    config.freq = I2C_FREQ_100K;
    I2C_Init(I2C_ACC, config);
    
    Trace(1,"i2c config init");
    printf("i2c config init");
       
    OS_Sleep(3000);
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) {
    /* TODO:IMPLEMENT or leave empty if no resources need to be freed */
    I2C_Error_t err = I2C_Close(I2C_ACC);
    if(err){
        Trace(1, "I2C Close failed. sensirion_i2c_hal_free unsuccessful! code: %d", err);
    } else {
        Trace(1, "I2C Close successful !!");
    }

}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */

/**
 * @a I2C_Receive
 * 
 * I2C_Error_t I2C_Receive(I2C_ID_t i2c, uint16_t slaveAddr, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * 
 * @param i2c：I2C ID
 * @param slaveAddr:slave address
 * @param pData:The location of the received data storage
 * @param length：length
 * @param timeOut:time out , unit: ms
 * 
 */

/** 
 * @a I2C_ReadMem
 * 
 * I2C_Error_t I2C_ReadMem(I2C_ID_t i2c, uint16_t slaveAddr, uint32_t memAddr, uint8_t memSize, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * 
 * @param i2c：I2C ID >> A9g te normally hobe @c I2C2 
 * @param slaveAddr:slave address >> SEN4x er jonno @c 0x69
 * @param memAddr：slave register address >> jei register call korte chai oitar address
 * @param memSize： slave register address length,unit:byte, max 4
 * @param pData: data read >> pointer hoile bhalo hoy
 * @param timeOut:time out , unit: ms >> datasheet theke alada register command er jonno alada hobe
 * 
 * 
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
    /* TODO:IMPLEMENT */
    // I2C_Error_t err = I2C_ReadMem(I2C_ACC, I2C_ADD, address, 1, &data, count, SENSIRION_I2C_DEFAULT_TIMEOUT);

    // uint8_t dataReading;
    // I2C_Error_t err = I2C_Receive(I2C_ACC, address, (uint8_t*)data, count, SENSIRION_I2C_DEFAULT_TIMEOUT);
   
    I2C_Error_t err = I2C_Receive(I2C2, address , data, count, SENSIRION_I2C_DEFAULT_TIMEOUT);
    if(err != I2C_ERROR_NONE){
        Trace(1, "I2C2 recieve err: %d", err);
    }    
    return err;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */


/**
 * @I2C_Error_t I2C_Transmit(I2C_ID_t i2c, uint16_t slaveAddr, uint8_t* pData, uint16_t length, uint32_t timeOut);
 * 
 * @param 2c：I2C ID
 * @param slaveAddr:slave address
 * @param pData: data to send
 * @param length：length
 * @param timeOut:time out , unit: ms
 * 
 */

/** 
 * 
 * I2C_Error_t I2C_WriteMem(I2C_ID_t i2c, uint16_t slaveAddr, uint32_t memAddr, uint8_t memSize, uint8_t* pData, uint16_t length, uint32_t timeOut);
 *
 * @param i2c：I2C ID >> A9g te normally hobe @c I2C2 
 * @param slaveAddr:slave address >> SEN4x er jonno @c 0x69
 * @param memAddr：slave register address >> jei register call korte chai oitar address
 * @param memSize： slave register address length,unit:byte, max 4
 * @param pData: data read >> pointer hoile bhalo hoy
 * @param length: length of data to write 
 * @param timeOut:time out , unit: ms >> datasheet theke alada register command er jonno alada hobe
 */

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
    /* TODO:IMPLEMENT */
    // I2C_Error_t err = I2C_WriteMem(I2C_ACC, I2C_ADD, address, 1, data, count, SENSIRION_I2C_DEFAULT_TIMEOUT);
    // uint8_t reg = *data;
    I2C_Error_t err = I2C_Transmit(I2C_ACC, address, data, count, SENSIRION_I2C_DEFAULT_TIMEOUT);
    if(err != I2C_ERROR_NONE)
    {
        Trace(1, "I2C2 write err: %d", err);
    }
    return err;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    uint32_t msec = useconds / 1000;
    if (useconds % 1000 > 0) {
        msec++;
    }
    OS_Sleep(msec);    
}
