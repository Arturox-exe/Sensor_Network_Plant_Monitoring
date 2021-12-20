/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <string.h>

#include "mbed.h"

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

//sensor includes
#include "mbed.h"
#include "MMA8451Q.h"
#include "TCS3472_I2C/TCS3472_I2C.h"
#include "MBed_Adafruit_GPS.h"


using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Sensor Variables declaration
 */
Adafruit_GPS myGPS(new BufferedSerial(PA_9, PA_10,9600)); //object of Adafruit's GPS class
extern uint32_t _rhData;
extern int32_t  _tData;
int rgb_readings[4]; // Declare a 4 element array to store RGB sensor readings
float result[3] = {0,0,0};

bool calculate;
bool I2CFinish;

Thread I2CThread(osPriorityNormal, 1024);
Thread GPSThread(osPriorityNormal, 1024);

bool GPSready;


// Sensor Functions
extern void readSensor(void);
extern bool RTHpresent();

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;
//static uint8_t DEV_EUI[] = { 0x7d, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t DEV_EUI[] = { 0x7F, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0xda };
static uint8_t APP_KEY[] = { 0xf3,0x1c,0x2e,0x8b,0xc6,0x71,0x28,0x1d,0x51,0x16,0xf0,0x8f,0xf0,0xb7,0x92,0x8f };
/**
 * Entry point for application
 */


char off[4] = "OFF";
char red[4] = "Red";
char green[6] = "Green";

DigitalOut Red(PH_0);
DigitalOut Green(PH_1);

AnalogIn light(PA_4);
AnalogIn moisture(PA_0);

char CalculateDominantColour(void){
			char DomColor;
									if(rgb_readings[1]<100 && rgb_readings[2]<100 && rgb_readings[3]<100) //If clear
										{
											DomColor = 'C';
										}
										else
											{
											if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>=rgb_readings[3]) //If max=RED
												{
													DomColor = 'R';
												}else if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]) //If max=Green
												{
													DomColor = 'G';
												}
												else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2])   //If max=Blue
														DomColor = 'B';						
												}
											
			return DomColor;
											}
void GPSread(void){
			myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
		Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 2000; //refresh time in ms
		GPSready = false;
		refresh_Timer.start();  //starts the clock on the timer
	
		printf("Connection established at 9600 baud...\r\n");
		char c;
	
		while(true){

			 c = myGPS.read();   //queries the GPS
		 //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {

            if ( !myGPS.parse(myGPS.lastNMEA())) {

                continue;
							
            }
        }
				
				if (std::chrono::duration_cast<std::chrono::milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
            refresh_Timer.reset();
						GPSready= true;
				}
			}
	
}
/******Function calculating the I2C part *******/
void I2CRead(void){
	TCS3472_I2C rgb_sensor (PB_9, PB_8);
	MMA8451Q acc(PB_9,PB_8,0x1c<<1);


	int present;
	while(true){
		//read of temperature and humidity first
		if(calculate){
			if(RTHpresent()){
					readSensor();
					//RTHerror = false;
			}
			present = acc.getWhoAmI();
			if(present == 0x1A){
				acc.getAccAllAxis(result);
			}
				rgb_sensor.enablePowerAndRGBC();
				rgb_sensor.getAllColors(rgb_readings);

			I2CFinish = true;
		}
		  
	}
}

int main(void)
{

	  I2CFinish = false;
		calculate = false;
		I2CThread.start(I2CRead);
		GPSThread.start(GPSread);
	

		Red = 1;
		Green = 1;
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }



	

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
    lorawan_connect_t connect_params;
		connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;
		
    retcode = lorawan.connect(connect_params);
		
    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();
		

    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    float latitude;
		float longitude;
		int16_t temperature;
		uint16_t humidity;
		char domColor;
	  uint16_t light_value = light.read_u16()*10000.00/65536.00;
		uint16_t moisture_value = moisture.read_u16()*10000.00/65536.00;
		
		uint8_t* pointer = tx_buffer;
		calculate = true;
		int i;
		while(!I2CFinish){
			printf(" ");
			};
		calculate = false;
		I2CFinish = false;
		temperature = _tData/10;
		humidity = _rhData/10;
		domColor = CalculateDominantColour();
	  
		char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

				if ((int)myGPS.fixquality > 0 && GPSready) {
							latitude = myGPS.latitude;
							longitude = myGPS.latitude;
            }else{
							latitude = (float)40.38952831294019;
							longitude = (float) -3.6289202549381536;
						}
						printf("\nDate/Hour: %d-%d-%d %d:%d:%d",myGPS.day,myGPS.month,myGPS.year,myGPS.hour,myGPS.minute,myGPS.seconds);
						printf("\nSatellites %d\n",myGPS.satellites);
					
		
		memcpy(pointer,&latitude, sizeof(latitude));
		pointer += sizeof(latitude);
		memcpy(pointer,&longitude,sizeof(longitude));
		pointer += sizeof(longitude);
		memcpy(pointer,&temperature,sizeof(temperature));
		pointer += sizeof(temperature);
		memcpy(pointer,&humidity,sizeof(humidity));
		pointer += sizeof(humidity);
		memcpy(pointer,&light_value,sizeof(light_value));
		pointer += sizeof(light_value);
		memcpy(pointer,&moisture_value,sizeof(moisture_value));
		pointer += sizeof(moisture_value);
		memcpy(pointer,&domColor,sizeof(domColor));
		pointer += sizeof(domColor);
		memcpy(pointer,&result[0],sizeof(result[0]));
		pointer += sizeof(result[0]);
		memcpy(pointer,&result[1],sizeof(result[1]));
		pointer += sizeof(result[1]);
		memcpy(pointer,&result[2],sizeof(result[2]));
		pointer += sizeof(result[2]);
		
		
    packet_len = 29;
		
		printf("\nLat: %f Long: %f\nTemp: %.02f oC, Hum: %.02f %RH, Light: %.02f %, Mois: %.02f %, domColor: %c, XYZ:%.02f,%.02f,%.02f",
					latitude,longitude,
					(float)temperature/100,(float)humidity/100,
						(float)light_value/100,(float)moisture_value/100,domColor,result[0],result[1],result[2]);
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);
		//retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
    //                       MSG_CONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
		
	
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);
		char string[30] = "";

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
				string[i] = rx_buffer[i];
			  
    }
    printf("\r\n");
		
		printf(" %s \n", string);
		
		if(strcmp (string, "Red") == 0) {
			Red = 0;
			Green = 1;
		}
		
		else if(strcmp (string, "Green") == 0 ){
			Red = 1;
			Green = 0;
		}
			
		else if(strcmp (string, "OFF") == 0) {
			Red = 1;
			Green = 1;
		
		}
		
		
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
