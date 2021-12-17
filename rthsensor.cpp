#include "mbed.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
 
/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
 
/** Si7013 Read Temperature Command */
#define SI7013_READ_TEMP_POST   0xE0 /* Read previous T data from RH measurement
                                        command*/
#define SI7013_READ_TEMP        0xE3 /* Stand-alone read temperature command */

/** Si7013 Read RH Command */
#define SI7013_READ_RH          0xE5 /* Perform RH (and T) measurement. */
/** Si7013 Read ID */
#define SI7013_READ_ID1_1       0xFA
#define SI7013_READ_ID1_2       0x0F
#define SI7013_READ_ID2_1       0xFc
#define SI7013_READ_ID2_2       0xc9
/** Si7013 Read Firmware Revision */
#define SI7013_READ_FWREV_1     0x84
#define SI7013_READ_FWREV_2     0xB8
 
/** I2C device address for Si7013 */
#define SI7013_ADDR      0x82
/** I2C device address for Si7021 */
#define SI7021_ADDR      0x80
 
 
/** Device ID value for Si7013 */
#define SI7013_DEVICE_ID 0x0D
/** Device ID value for Si7020 */
#define SI7020_DEVICE_ID 0x14
/** Device ID value for Si7021 */
#define SI7021_DEVICE_ID 0x15
 
/** @endcond */

/*******************************************************************************
 *******************************  Variables  ***********************************
 ******************************************************************************/

//LED1
DigitalOut myled(LED1);
//I2C-bus with RHT sensor (Si7021) on Happy Gecko STK
I2C tempSensor(PB_9, PB_8);
//And enable line for the sensor
DigitalOut SENS_EN(D13);

uint8_t  _address = 0;
uint8_t  _rx_buf[8] = {0};
uint8_t  _tx_buf[2] = {0};
 
uint32_t _rhData = 0;
int32_t  _tData = 0;

/*******************************************************************************
 *******************************     FUNC    ***********************************
 ******************************************************************************/

void readSensor(void) {
    int temp = 0;
    unsigned int humidity = 0;
    
    //send humidity command
    _tx_buf[0] = SI7013_READ_RH;
    tempSensor.write(_address, (char*)_tx_buf, 1);
    tempSensor.read(_address, (char*)_rx_buf, 2);
    
    /* Store raw RH info */
    humidity = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert value to milli-percent */
    humidity = (((humidity) * 15625L) >> 13) - 6000;
    
    //send temperature command
    _tx_buf[0] = SI7013_READ_TEMP_POST;
    tempSensor.write(_address, (char*)_tx_buf, 1);
    tempSensor.read(_address, (char*)_rx_buf, 2);
    
    /* Store raw temperature info */
    temp = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert to milli-degC */
    temp = (((temp) * 21965L) >> 13) - 46850;
    
    _tData = temp;
    _rhData = humidity;
} 

bool RTHpresent() {
    //Enable the sensor
		bool sensorpresent = true;
    SENS_EN = 1;
    
    //Check if the sensor is present
    _tx_buf[0] = SI7013_READ_ID2_1;
    _tx_buf[1] = SI7013_READ_ID2_2;
    
    _address = SI7021_ADDR; //TODO: update if we use another sensor
    
    tempSensor.write(_address, (char*)_tx_buf, 2);
    tempSensor.read(_address, (char*)_rx_buf, 8);
    //Check ID byte
    if(_rx_buf[0] != SI7021_DEVICE_ID) {
        sensorpresent = false;
    }
    return sensorpresent;
}