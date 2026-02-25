/*
 * Copyright (C) 2016-2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************************************

   Application Name     -   MQTT Client
   Application Overview -   The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           message can be published on pre-configured topics
                           by pressing the switch buttons on the device.

   Application Details  - Refer to 'MQTT Client' README.html

*****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <mqueue.h>
#include <assert.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>
#include <ti/drivers/net/wifi/slwificonn.h>
#include <ti/net/slnet.h>
#include <ti/net/slnetif.h>
#include <ti/net/slnetconn.h>
#include <ti/net/mqtt/mqttclient.h>


#include "ifmod/uart_if.h"
#include "ifmod/wifi_if.h"
#include "ifmod/mqtt_if.h"
#include "ifmod/debug_if.h"
#include "ifmod/ota_if.h"
#include "ifmod/utils_if.h"
#include "ifmod/ota_vendors.h"

#include "ti_drivers_config.h"

#include "aws_iot/backoff_algorithm.h"

#define USE_HEATSHRINK
#undef USE_HEATSHRINK
#ifdef USE_HEATSHRINK
#include <heatshrink/heatshrink_encoder.h>
#include <heatshrink/heatshrink_decoder.h>
#include <heatshrink/heatshrink_common.h>
#include <heatshrink/heatshrink_config.h>
#endif  /* USE_HEATSHRINK */

#define USE_LZ4
#undef USE_LZ4
#ifdef USE_LZ4
#include "lz4/lz4.h"
#endif

#undef DEBUG_IF_NAME
#define DEBUG_IF_NAME       "MQTT_APP"
#undef DEBUG_IF_SEVERITY
#define DEBUG_IF_SEVERITY   E_TRACE



/* Add some backoff algo stuff... */

/* The maximum number of retries for the example code. */
#define RETRY_MAX_ATTEMPTS            ( 10U )

/* The maximum back-off delay (in milliseconds) for between retries in the example. */
#define RETRY_MAX_BACKOFF_DELAY_MS    ( 50000U )

/* The base back-off delay (in milliseconds) for retry configuration in the example. */
#define RETRY_BACKOFF_BASE_MS         ( 1000U )


#ifdef USE_HEATSHRINK
/* Add some heatshrink stuff */

#define BUFFER_SIZE 10056
uint8_t orig_buffer[BUFFER_SIZE];
uint8_t comp_buffer[BUFFER_SIZE];
uint8_t decomp_buffer[BUFFER_SIZE];

#if HEATSHRINK_DYNAMIC_ALLOC
#error HEATSHRINK_DYNAMIC_ALLOC must be false for static allocation test suite.
#endif

#define HEATSHRINK_DEBUG

static heatshrink_encoder hse;
static heatshrink_decoder hsd;

#endif  /* USE_HEATSHRINK */

extern int32_t ti_net_SlNet_initConfig();

extern void startSNTP(void);

static void startOTA(char* topic, char* payload, uint8_t qos);
static void StartCloudOTA();
static void StartLocalOTA();
static void StartInternalUpdate();

#ifdef USE_HEATSHRINK
/* Reset an encoder. */
extern void heatshrink_encoder_reset(heatshrink_encoder *hse);

/* Sink up to SIZE bytes from IN_BUF into the encoder.
 * INPUT_SIZE is set to the number of bytes actually sunk (in case a
 * buffer was filled.). */
extern HSE_sink_res heatshrink_encoder_sink(heatshrink_encoder *hse,
    uint8_t *in_buf, size_t size, size_t *input_size);

/* Poll for output from the encoder, copying at most OUT_BUF_SIZE bytes into
 * OUT_BUF (setting *OUTPUT_SIZE to the actual amount copied). */
extern HSE_poll_res heatshrink_encoder_poll(heatshrink_encoder *hse,
    uint8_t *out_buf, size_t out_buf_size, size_t *output_size);

/* Notify the encoder that the input stream is finished.
 * If the return value is HSER_FINISH_MORE, there is still more output, so
 * call heatshrink_encoder_poll and repeat. */
extern HSE_finish_res heatshrink_encoder_finish(heatshrink_encoder *hse);

#ifdef HEATSHRINK_DEBUG
#ifdef HEATSHRINK_DUMP_ENABLED
static void dump_buf(const char *name, uint8_t *buf, uint16_t count) {
    for (int i=0; i<count; i++) {
        uint8_t c = (uint8_t)buf[i];
        UART_PRINT("%s %d: 0x%02x ('%c')\n", name, i, c, isprint(c) ? c : '.');
    }
}
#else
static void dump_buf(const char *name, uint8_t *buf, uint16_t count) {
    // do nothing
}
#endif
#endif

//static void compress(uint8_t *dataIn, uint32_t sizeDataIn, uint8_t *dataOut, uint32_t &sizeDataOut) {
static void compress(uint8_t *dataIn, uint32_t sizeDataIn, uint8_t *dataOut, uint32_t sizeDataOut) {

    heatshrink_encoder_reset(&hse);

    #ifdef HEATSHRINK_DEBUG
    UART_PRINT("\n^^ COMPRESSING\n");
    dump_buf("input", dataIn, sizeDataIn);
    #endif

    size_t   count  = 0;
    uint32_t sunk   = 0;
    uint32_t polled = 0;
    while (sunk < sizeDataIn) {
        //ASSERT(heatshrink_encoder_sink(&hse, &dataIn[sunk], sizeDataIn - sunk, &count) >= 0);
        heatshrink_encoder_sink(&hse, &dataIn[sunk], sizeDataIn - sunk, &count);
        sunk += count;
        #ifdef HEATSHRINK_DEBUG
        UART_PRINT("^^ sunk = %d\n\r", count);
        #endif
        if (sunk == sizeDataIn) {
            //ASSERT_EQ(HSER_FINISH_MORE, heatshrink_encoder_finish(&hse));
            heatshrink_encoder_finish(&hse);
        }

        HSE_poll_res pres;
        do {                    /* "turn the crank" */
            pres = heatshrink_encoder_poll(&hse,
                                           &dataOut[polled],
                                           sizeDataOut - polled,
                                           &count);
            //ASSERT(pres >= 0);
            polled += count;
            #ifdef HEATSHRINK_DEBUG
            UART_PRINT("^^ polled: %d\n\r",  polled);
            #endif
        } while (pres == HSER_POLL_MORE);
        //ASSERT_EQ(HSER_POLL_EMPTY, pres);
        #ifdef HEATSHRINK_DEBUG
        if (polled >= sizeDataOut){
            UART_PRINT("FAIL: Exceeded the size of the output buffer!\n\r");
        }
        #endif
        if (sunk == sizeDataIn) {
            //ASSERT_EQ(HSER_FINISH_DONE, heatshrink_encoder_finish(&hse));
            heatshrink_encoder_finish(&hse);
        }
    }
    #ifdef HEATSHRINK_DEBUG
    UART_PRINT("sizeDataIn: %d | compressed: %d | polled (sizeDataOut): %d\n\r", sizeDataIn, polled );
    #endif
    //update the output size to the (smaller) compressed size
    sizeDataOut = polled;

    #ifdef HEATSHRINK_DEBUG
    dump_buf("output", dataOut, sizeDataOut);
    #endif
}

static void decompress(uint8_t *dataIn, uint32_t sizeDataIn, uint8_t *dataOut, uint32_t sizeDataOut) {

    heatshrink_decoder_reset(&hsd);
    #ifdef HEATSHRINK_DEBUG
    UART_PRINT("\n^^ DECOMPRESSING\n");
    dump_buf("input", dataIn, sizeDataIn);
    #endif
    size_t   count  = 0;
    uint32_t sunk   = 0;
    uint32_t polled = 0;
    while (sunk < sizeDataIn) {
        //ASSERT(heatshrink_decoder_sink(&hsd, &comp[sunk], input_size - sunk, &count) >= 0);
        heatshrink_decoder_sink(&hsd, &dataIn[sunk], sizeDataIn - sunk, &count);
        sunk += count;
        #ifdef HEATSHRINK_DEBUG
        UART_PRINT("^^ sunk: %d\n\r", count);
        #endif
        if (sunk == sizeDataIn) {
            //ASSERT_EQ(HSDR_FINISH_MORE, heatshrink_decoder_finish(&hsd));
            heatshrink_decoder_finish(&hsd);
        }

        HSD_poll_res pres;
        do {
            pres = heatshrink_decoder_poll(&hsd, &dataOut[polled], sizeDataOut - polled, &count);
            //ASSERT(pres >= 0);
            polled += count;
            #ifdef HEATSHRINK_DEBUG
            UART_PRINT("^^ polled: %d\n\r", polled);
            #endif
        } while (pres == HSDR_POLL_MORE);
        //ASSERT_EQ(HSDR_POLL_EMPTY, pres);
        if (sunk == sizeDataIn) {
            HSD_finish_res fres = heatshrink_decoder_finish(&hsd);
            //ASSERT_EQ(HSDR_FINISH_DONE, fres);
        }
        if (polled > sizeDataOut) {
            #ifdef HEATSHRINK_DEBUG
            UART_PRINT("FAIL: Exceeded the size of the output buffer!\n\r");
            #endif
        }
    }
    #ifdef HEATSHRINK_DEBUG
    UART_PRINT("sizeDataIn: %d | decompressed: %d\n\r", sizeDataIn, polled);
    #endif
    //update the output size
    sizeDataOut = polled;

    #ifdef HEATSHRINK_DEBUG
    dump_buf("output", dataOut, sizeDataOut);
    #endif
}

#endif  /* USE_HEATSHRINK */

pthread_t gSlNetConnThread;
bool gNewImageLoaded = false;
#define SLNETCONN_TIMEOUT               10 // 10 Second Timeout
#define SLNETCONN_TASK_STACK_SIZE       (2048)


#define APPLICATION_NAME         "MQTT client"
#define APPLICATION_VERSION      "2.0.3"

// un-comment this if you want to connect to an MQTT broker securely
#define MQTT_SECURE_CLIENT

#define MQTT_MODULE_TASK_PRIORITY   2
#define MQTT_MODULE_TASK_STACK_SIZE 8192

#define MQTT_WILL_TOPIC             "cc32xx_will_topic"
#define MQTT_WILL_MSG               "will_msg_works"
#define MQTT_WILL_QOS               MQTT_QOS_0
#define MQTT_WILL_RETAIN            false


#define MQTT_CLIENT_PASSWORD        NULL
#define MQTT_CLIENT_USERNAME        NULL
#define MQTT_CLIENT_KEEPALIVE       300
#define MQTT_CLIENT_CLEAN_CONNECT   true
#define MQTT_CLIENT_MQTT_V3_1       false
#define MQTT_CLIENT_BLOCKING_SEND   false

#ifdef MQTT_SECURE_CLIENT

#define MQTT_CONNECTION_ADDRESS         "afw7ct4mv25wq-ats.iot.us-east-1.amazonaws.com"
#define MQTT_CONNECTION_PORT_NUMBER     8883
#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_URL | MQTTCLIENT_NETCONN_SEC \
                                        | MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION

/*
* In order to connect to an MQTT broker securely, the MQTTCLIENT_NETCONN_SEC flag,
* method for secure socket, cipher, secure files, number of secure files must be set
* and the certificates must be programmed to the file system.
*
* The first parameter is a bit mask which configures the server address type and security mode.
* Server address type: IPv4, IPv6 and URL must be declared with the corresponding flag.
* All flags can be found in mqttclient.h.
*
* The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS) which includes domain name
* verification and certificate catalog verification. Those verifications can be skipped by
* adding to the bit mask: MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and
* MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION.
*
* Note: The domain name verification requires URL Server address type otherwise, this
* verification will be disabled.
*/

#else

#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_IP4
#define MQTT_CONNECTION_ADDRESS         "192.168.2.8"
// #define MQTT_CONNECTION_FLAGS        MQTTCLIENT_NETCONN_URL
//#define MQTT_CONNECTION_ADDRESS       "broker.hivemq.com" //"mqtt.eclipse.org"
#define MQTT_CONNECTION_PORT_NUMBER     1883

#endif

#define OTA_DEFAULT_METHOD              StartCloudOTA
//#define OTA_DEFAULT_METHOD              StartLocalOTA
//#define OTA_DEFAULT_METHOD              StartInternalUpdate

struct sensorData {                     // total size in bytes = 2,656 bytes for every second of data

    // header = 32 bytes
    char            magic[10];           // size = 8 bytes, always 'collar10' for collarOS v1.0 data structure
    char            macAddr[6];         // size = 6 bytes, varies by device
    uint32_t        blockNumber;        // size = 4 bytes, varies, auto-incrementing, then restarting at 0 after 16.whatever million
    time_t          timeStamp;          // size = 4 bytes, varies, auto-incrementing, should always increase by 5 seconds from last to next
    char            startBlock[8];      // size = 8 bytes, always '|+data+|'

    // Optics = 4,000 bytes              // size = always 4,000 bytes
    int32_t         led1[250];           // size = 200 bytes; varies, except for simulated data
    int32_t         led2[250];           // size = 200 bytes; varies, except for simulated data
    int32_t         led3[250];           // size = 200 bytes; varies, except for simulated data
    int32_t         led4[250];           // size = 200 bytes; varies, except for simulated data
    // IMU = 3,000 bytes                 // size = always 3,000 bytes for A and G data
    int16_t         ax[250];             // size = 200 bytes; varies, except for simulated data
    int16_t         ay[250];             // size = 200 bytes; varies, except for simulated data
    int16_t         az[250];             // size = 200 bytes; varies, except for simulated data
    int16_t         gx[250];             // size = 200 bytes; varies, except for simulated data
    int16_t         gy[250];             // size = 200 bytes; varies, except for simulated data
    int16_t         gz[250];             // size = 200 bytes; varies, except for simulated data
    // PRH = 3,000 bytes                 // always 3,000 bytes for PRH
    float           pitch[250];          // size = 200 bytes; varies, except for simulated data
    float           roll[250];           // size = 200 bytes; varies, except for simulated data
    float           heading[250];        // size = 200 bytes; varies, except for simulated data

    // endBlock                         // always 16 bytes
    char            endData[8];        // always '|-data-|'
    char            endBlock[8];       // always 'endBlock'

} realData, simData;

char magic[10]      = "collar1000";
char startBlock[8]  = "|+Data+|";
char endData[8]     = "|-Data-|";
char endBlock[8]    = "endBlock";


char macAddr[12]      = {0};


mqd_t appQueue;
volatile int connected;
volatile int deinit;
Timer_Handle timer0;
volatile int longPress = 0;

/* Client ID - if ClientId isn't set,
 * the MAC address of the device will be copied
 * into the ClientID parameter.
 */

char ClientId[6] = {0};

enum{
    APP_MQTT_PUBLISH,
    APP_MQTT_CON_TOGGLE,
    APP_MQTT_DEINIT,
    APP_BTN_HANDLER,
    APP_OTA_TRIGGER,
    APP_SEND_DATA_TO_AWS_TRIGGER
};

struct msgQueue
{
    int   event;
    char* payload;
};

MQTT_IF_InitParams_t mqttInitParams =
{
     MQTT_MODULE_TASK_STACK_SIZE,   // stack size for mqtt module - default is 2048
     MQTT_MODULE_TASK_PRIORITY      // thread priority for MQTT   - default is 2
};

MQTTClient_Will mqttWillParams =
{
     MQTT_WILL_TOPIC,    // will topic
     MQTT_WILL_MSG,      // will message
     MQTT_WILL_QOS,      // will QoS
     MQTT_WILL_RETAIN    // retain flag
};

MQTT_IF_ClientParams_t mqttClientParams =
{
     ClientId,                  // client ID
     MQTT_CLIENT_USERNAME,      // user name
     MQTT_CLIENT_PASSWORD,      // password
     MQTT_CLIENT_KEEPALIVE,     // keep-alive time
     MQTT_CLIENT_CLEAN_CONNECT, // clean connect flag
     MQTT_CLIENT_MQTT_V3_1,     // true = 3.1, false = 3.1.1
     MQTT_CLIENT_BLOCKING_SEND, // blocking send flag
     &mqttWillParams            // will parameters
};

#ifndef MQTT_SECURE_CLIENT
MQTTClient_ConnParams mqttConnParams =
{
     MQTT_CONNECTION_FLAGS,         // connection flags
     MQTT_CONNECTION_ADDRESS,       // server address
     MQTT_CONNECTION_PORT_NUMBER,   // port number of MQTT server
     0,                             // method for secure socket
     0,                             // cipher for secure socket
     0,                             // number of files for secure connection
     NULL                           // secure files
};
#else

/*
 * Secure clients require time configuration in order to verify the server certificate validity (date)
 */

/* Day of month (DD format) range 1-31                                       */
#define DAY                      7
/* Month (MM format) in the range of 1-12                                    */
#define MONTH                    2
/* Year (YYYY format)                                                        */
#define YEAR                     2024
/* Hours in the range of 0-23                                                */
#define HOUR                     4
/* Minutes in the range of 0-59                                              */
#define MINUTES                  15
/* Seconds in the range of 0-59                                              */
#define SEC                      20





/** Secure Socket Parameters to open a secure connection */
/*
 Note: value of n_files can vary from 1 to 4, with corresponding pointers to
 the files in files field. Value of 1(n_files) will assume the corresponding
 pointer is for CA File Name. Any other value of n_files expects the files to
 be in following order:
 1.  Private Key File
 2.  Certificate File Name
 3.  CA File Name
 4.  DH Key File Name

 example:
 If you want to provide only CA File Name, following are the two way of doing it:
 for n_files = 1
 char *security_file_list[] = {"/cert/testcacert.der"};
 for n_files = 4
 char *security_file_list[] = {NULL, NULL, "/cert/testcacert.der", NULL};
 where secure_files = security_file_list
 */

char *MQTTClient_secureFiles[4] = { "FC_00000209_privkey.pem", "FC_00000209.pem", "sf-class2-root.pem", NULL };


MQTTClient_ConnParams mqttConnParams =
{
    MQTT_CONNECTION_FLAGS,                  // connection flags
    MQTT_CONNECTION_ADDRESS,                // server address
    MQTT_CONNECTION_PORT_NUMBER,            // port number of MQTT server
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,     // method for secure socket
    SLNETSOCK_SEC_CIPHER_FULL_LIST,         // cipher for secure socket
    4,                                      // number of files for secure connection
    MQTTClient_secureFiles                  // secure files
};

void setTime() {

    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}

#endif

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if ( pSock->Event == SL_SOCKET_ASYNC_EVENT)
    {
        switch (pSock->SocketAsyncEvent.SockAsyncData.Type)
        {
        case SL_SSL_NOTIFICATION_WRONG_ROOT_CA:
            /* on socket error Restart OTA */
            LOG_INFO("SL_SOCKET_ASYNC_EVENT: ERROR - WRONG ROOT CA");
            LOG_INFO("Please install the following Root Certificate:");
            LOG_INFO(" %s\n\r", pSock->SocketAsyncEvent.SockAsyncData.pExtraInfo);
            break;
        default:
            /* on socket error Restart OTA */
            LOG_INFO("SL_SOCKET_ASYNC_EVENT socket event %d", pSock->Event);
            LOG_INFO("%s\n\r", pSock->SocketAsyncEvent.SockAsyncData.pExtraInfo);
        }
    }
}

void SimpleLinkHttpServerEventHandler(
        SlNetAppHttpServerEvent_t *pHttpEvent,
        SlNetAppHttpServerResponse_t *
        pHttpResponse)
{
    /* Unused in this application */
}

void SimpleLinkNetAppRequestMemFreeEventHandler(_u8 *buffer)
{
    /* Unused in this application */
}

//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret = 0;
    uint8_t Client_Mac_Name[2];
    uint8_t Index;
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint8_t macAddress[SL_MAC_ADDR_LEN];

    /*Get the device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    /*When ClientID isn't set, use the mac address as ClientID               */
    if(ClientId[0] == '\0')
    {
        /*6 bytes is the length of the mac address                           */
        for(Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /*Each mac address byte contains two hexadecimal characters      */
            /*Copy the 4 MSB - the most significant character                */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /*Copy the 4 LSB - the least significant character               */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if(Client_Mac_Name[0] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if(Client_Mac_Name[1] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
    return(ret);
}

void timerCallback(Timer_Handle myHandle)
{
    longPress = 1;
}

// this timer callback toggles the LED once per second until the device connects to an AP
void timerLEDCallback(Timer_Handle myHandle)
{
    GPIO_toggle(CONFIG_GPIO_LED_GREEN_GPIO11);
}

void pushDataToAws(char* topic, char* payload, uint8_t qos){

    GPIO_write(CONFIG_GPIO_LED_YELLOW_GPIO10, 1);

    int ret;
    struct msgQueue queueElement;

    queueElement.event = APP_MQTT_PUBLISH;
    ret = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
    if(ret < 0){
        LOG_ERROR("msg queue send error %d", ret);
    }

}

void pushButtonPublishHandler(uint_least8_t index)
{
    int ret;
    struct msgQueue queueElement;

    GPIO_disableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);

    queueElement.event = APP_MQTT_PUBLISH;
    ret = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
    if(ret < 0){
        LOG_ERROR("msg queue send error %d", ret);
    }

//    queueElement.event = APP_OTA_TRIGGER;
//    queueElement.event = APP_MQTT_PUBLISH;
//    ret = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
//    if(ret < 0){
//        LOG_ERROR("msg queue send error %d", ret);
//    }

}

void pushButtonConnectionHandler(uint_least8_t index)
{
    int ret;
    struct msgQueue queueElement;

    GPIO_disableInt(CONFIG_GPIO_BUTTON_SW2_LEFT);

    ret = Timer_start(timer0);
    if(ret < 0){
        LOG_ERROR("failed to start the timer\r\n");
    }

    queueElement.event = APP_BTN_HANDLER;

    ret = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
    if(ret < 0){
        LOG_ERROR("msg queue send error %d", ret);
    }
}

int detectLongPress(){

    int buttonPressed;

    do{
        buttonPressed = GPIO_read(CONFIG_GPIO_BUTTON_SW3_RIGHT);
    }while(buttonPressed && !longPress);

    // disabling the timer in case the callback has not yet triggered to avoid updating longPress
    Timer_stop(timer0);

    if(longPress == 1){
        longPress = 0;
        return 1;
    }
    else{
        return 0;
    }
}


void MQTT_EventCallback(int32_t event){

    struct msgQueue queueElement;

    switch(event){

        case MQTT_EVENT_CONNACK:
        {
            deinit = 0;
            connected = 1;
            LOG_INFO("MQTT_EVENT_CONNACK\r\n");
            GPIO_clearInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
            GPIO_enableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
            break;
        }

        case MQTT_EVENT_SUBACK:
        {
            LOG_INFO("MQTT_EVENT_SUBACK\r\n");
            break;
        }

        case MQTT_EVENT_PUBACK:
        {
            LOG_INFO("MQTT_EVENT_PUBACK\r\n");
            break;
        }

        case MQTT_EVENT_UNSUBACK:
        {
            LOG_INFO("MQTT_EVENT_UNSUBACK\r\n");
            break;
        }

        case MQTT_EVENT_CLIENT_DISCONNECT:
        {
            connected = 0;
            LOG_INFO("MQTT_EVENT_CLIENT_DISCONNECT\r\n");
            if(deinit == 0){
                GPIO_clearInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
                GPIO_enableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
            }
            break;
        }

        case MQTT_EVENT_SERVER_DISCONNECT:
        {
            connected = 0;

            LOG_INFO("MQTT_EVENT_SERVER_DISCONNECT\r\n");

            queueElement.event = APP_MQTT_CON_TOGGLE;
            int res = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
            if(res < 0){
                LOG_ERROR("msg queue send error %d", res);
            }
            break;
        }

        case MQTT_EVENT_DESTROY:
        {
            LOG_INFO("MQTT_EVENT_DESTROY\r\n");
            break;
        }
    }
}

/*
 * Subscribe topic callbacks
 * Topic and payload data is deleted after topic callbacks return.
 * User must copy the topic or payload data if it needs to be saved.
 */
void awsMainBroker(char* topic, char* payload, uint8_t qos){

    LOG_INFO("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);

}

void publishSimData(char* topic, char* payload, uint8_t qos){

}

void toggleGreenLedCallback(char* topic, char* payload, uint8_t qos){

    GPIO_toggle(CONFIG_GPIO_LED_GREEN_GPIO11);
    LOG_INFO("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);

}

void toggleYellowLedCallback(char* topic, char* payload, uint8_t qos){

    GPIO_toggle(CONFIG_GPIO_LED_YELLOW_GPIO10);
    LOG_INFO("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);

}

void toggleRedLedCallback(char* topic, char* payload, uint8_t qos){
    GPIO_toggle(CONFIG_GPIO_LED_RED_GPIO09);
    LOG_INFO("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);
}

void publishSimDataCallback(char* topic, char* payload, uint8_t qos){


    LOG_INFO("Topic: %s | Payload: %s | QOS: %d\n\r", topic, payload, qos);

}


int32_t DisplayAppBanner(char* appName, char* appVersion){

    int32_t ret = 0;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t ConfigSize = 0;
    uint8_t ConfigOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    ConfigSize = sizeof(SlDeviceVersion_t);

    // get the device version info and MAC address
    ret = sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize, (uint8_t*)(&ver));
    ret |= (int32_t)sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen, &macAddress[0]);

    UART_PRINT("\n\r\t============================================\n\r");
    UART_PRINT("\t   %s Example Ver: %s",appName, appVersion);
    UART_PRINT("\n\r\t============================================\n\r\n\r");

    UART_PRINT("\t CHIP: 0x%x\n\r",ver.ChipId);
    UART_PRINT("\t MAC:  %d.%d.%d.%d\n\r",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT("\t PHY:  %d.%d.%d.%d\n\r",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT("\t NWP:  %d.%d.%d.%d\n\r",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT("\t ROM:  %d\n\r",ver.RomVersion);
    UART_PRINT("\t HOST: %s\n\r", SL_DRIVER_VERSION);
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT("\n\r\t============================================\n\r");

    return(ret);
}

//*****************************************************************************
//
//! \brief  SlWifiConn Event Handler
//!
//*****************************************************************************
static void SlNetConnEventHandler(uint32_t ifID, SlNetConnStatus_e netStatus, void* data)
{
    switch(netStatus)
    {
    case SLNETCONN_STATUS_CONNECTED_MAC:
        UART_PRINT("[SlNetConnEventHandler] I/F %d - CONNECTED (MAC LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_CONNECTED_IP:
        UART_PRINT("[SlNetConnEventHandler] I/F %d - CONNECTED (IP LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_CONNECTED_INTERNET:
        UART_PRINT("[SlNetConnEventHandler] I/F %d - CONNECTED (INTERNET LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_WAITING_FOR_CONNECTION:
    case SLNETCONN_STATUS_DISCONNECTED:
        UART_PRINT("[SlNetConnEventHandler] I/F %d - DISCONNECTED!\n\r", ifID);
    break;
    default:
        UART_PRINT("[SlNetConnEventHandler] I/F %d - UNKNOWN STATUS\n\r", ifID);
    break;
    }
}

#if OTA_SUPPORT

void OtaCallback(otaNotif_e notification, OtaEventParam_u *pParams)
{

    SlNetConnStatus_e status;
    int retVal;

    switch(notification)
    {
    case OTA_NOTIF_IMAGE_PENDING_COMMIT:
         LOG_INFO("OTA_NOTIF_IMAGE_PENDING_COMMIT");
         retVal = SlNetConn_getStatus(true, &status);
         if (retVal == 0 && status == SLNETCONN_STATUS_CONNECTED_INTERNET)
         {
             OTA_IF_commit();
         }
         else
         {
             OTA_IF_rollback();
             LOG_ERROR("Error Testing the new version - reverting to old version (%d)", retVal);
         }
         break;
    case OTA_NOTIF_IMAGE_DOWNLOADED:
    {
        struct msgQueue queueElement;
        gNewImageLoaded = true;
        LOG_INFO("OTA_NOTIF_IMAGE_DOWNLOADED");
        /* Closing the MQTT - will take the main thread out of its execution loop, then
         * the gNewImageLoaded will mark that new Update is ready for installation
         * (which involves MCU Reset)
         */
        queueElement.event = APP_MQTT_DEINIT;
        retVal = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
        assert (retVal == 0);
        break;
    }
    case OTA_NOTIF_GETLINK_ERROR:
        LOG_ERROR("OTA_NOTIF_GETLINK_ERROR (%d)", pParams->err.errorCode);
        break;
    case OTA_NOTIF_DOWNLOAD_ERROR:
        LOG_ERROR("OTA_NOTIF_DOWNLOAD_ERROR (%d)", pParams->err.errorCode);
        break;
    case OTA_NOTIF_INSTALL_ERROR:
        LOG_ERROR("OTA_NOTIF_INSTALL_ERROR (%d)", pParams->err.errorCode);
        break;
   case OTA_NOTIF_COMMIT_ERROR:
         LOG_ERROR("OTA_NOTIF_COMMIT_ERROR {%d)", pParams->err.errorCode);
         break;
    default:
        LOG_INFO("OTA_NOTIF not handled Id %d", notification);
        break;
    }
}
#endif

static void StartCloudOTA()
{
#if CLOUD_OTA_SUPPORT
    LOG_INFO("Starting Cloud OTA");
//    OTA_IF_downloadImageByCloudVendor(OTA_GITHUB_getDownloadLink, OTA_DROPBOX_getDownloadLink, 0);
    OTA_IF_downloadImageByCloudVendor(OTA_GITHUB_getDownloadLink, 0, 0);
#else
    LOG_WARNING("Cloud OTA is not enabled. Please enable CLOUD_OTA_SUPPORT macro in ota_settings.h");
#endif
}

static void StartLocalOTA()
{
#if LOCAL_OTA_SUPPORT
    LOG_INFO("Starting Local OTA");
    OTA_IF_uploadImage(0); // use default HTTP port and no security
#else
    LOG_WARNING("Local OTA is not enabled. Enable LOCAL_OTA_SUPPORT macro. Please enable CLOUD_OTA_SUPPORT macro in ota_settings.h");
#endif
}

static void StartInternalUpdate()
{
#if INTERNAL_UPDATE_SUPPORT
    TarFileParams_t tarFileParams;

    tarFileParams.pPath =  "/otaImages/cc3235sf.tar";
    tarFileParams.token = 0;
    tarFileParams.pVersion = NULL;
    LOG_INFO("Starting Internal Update");
    OTA_IF_readImage(&tarFileParams, 0); // use default HTTP port and no security
#else
    LOG_WARNING("Internal Update is not enabled. Please enable INTERNAL_UPDATE_SUPPORT macro in ota_settings.h");
#endif
}

static void startOTA(char* topic, char* payload, uint8_t qos)
{
    if(topic != NULL)
    {
        LOG_INFO("Trigger OTA...(MQTT:: %s)", payload);
        if(strcmp(payload, "cloud") == 0)
        {
            StartCloudOTA();
        }
        else if(strcmp(payload, "local") == 0)
        {
            StartLocalOTA();
        }
        else if(strcmp(payload, "internal") == 0)
        {
            StartInternalUpdate();
        }
        else
        {
            /* Unknown payload - mark as NULL to use the default method */
            LOG_WARNING("Unknown payload - using the default OTA method");
            topic = NULL;
        }
    }
    else
    {
        LOG_INFO("Trigger OTA...(button press)");
    }
    /* The following is the default for button press or unknown MQTT payload */
    if(topic == NULL)
    {
#ifdef OTA_DEFAULT_METHOD
        OTA_DEFAULT_METHOD();
#else
        LOG_WARNING("No default OTA method is defined.");
        LOG_WARNING("Set OTA_DEFAULT_METHOD to one of: StartCloudOTA / StartLocalOTA / StartInternalUpdate");

#endif
    }
}

void mainThread(void * args) {

    uint32_t    pubFail_minus5 = 0;
    uint32_t    republishSuccessOnFailure = 0;
    int32_t ret;
    int32_t conn_Failure = 0;
    mq_attr attr;
    Timer_Params params;
    struct msgQueue queueElement;
    MQTTClient_Handle mqttClientHandle = NULL;

    // Some structs and data types for building simulated sensorData

    struct sensorData   simData;
    time_t t;

    float a = 64.1;

    /* Backoff algo stuff */

    BackoffAlgorithmStatus_t retryStatus = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t retryParams;
    uint16_t nextRetryBackoff = 0;

    /* Initialize reconnect attempts and interval. */
    BackoffAlgorithm_InitializeParams( &retryParams,
                                       RETRY_BACKOFF_BASE_MS,
                                       RETRY_MAX_BACKOFF_DELAY_MS,
                                       RETRY_MAX_ATTEMPTS );

    InitTerm();

    GPIO_init();
    SPI_init();
    Timer_init();

    GPIO_setCallback(CONFIG_GPIO_BUTTON_SW3_RIGHT, pushButtonPublishHandler);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_SW2_LEFT, pushButtonConnectionHandler);

    GPIO_write(CONFIG_GPIO_LED_GREEN_GPIO11, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_LED_YELLOW_GPIO10, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_LED_RED_GPIO09, CONFIG_GPIO_LED_ON);

    // configuring the timer to toggle an LED until the AP is connected
    Timer_Params_init(&params);
    params.period = 1500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.timerCallback = (Timer_CallBackFxn)timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        LOG_ERROR("failed to initialize timer\r\n");
        while(1);
    }

    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(struct msgQueue);
    appQueue = mq_open("appQueue", O_CREAT, 0, &attr);
    if(((int)appQueue) <= 0){
        while(1);
    }

    /* Enable SlNet framework */
    ret = ti_net_SlNet_initConfig();
    if(0 != ret)
    {
        LOG_ERROR("Failed to initialize SlNetSock\n\r");
    }

    /* Enable SlWifiConn */
    ret = WIFI_IF_init();
    assert (ret == 0);

    /* To get the NWP content (version + MAC),
     * The DisplayAppBanner should be called after WIFI_IF_init() after sl_Start() */
    DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);


    /* Enable SlNetConn */
    ret = SlNetConn_init(0);
    assert (ret == 0);

    gSlNetConnThread = OS_createTask(1, SLNETCONN_TASK_STACK_SIZE, SlNetConn_process, NULL, OS_TASK_FLAG_DETACHED);
    assert(gSlNetConnThread);

    
    ret = SlNetConn_start(SLNETCONN_SERVICE_LVL_INTERNET, SlNetConnEventHandler, SLNETCONN_TIMEOUT, 0);

    /* If Failure to acquire AP Connection, verify no pending OTA commit by initializing OTA library then return to attempting to connect to the AP */
    if(ret != 0)
    {
        LOG_INFO("failed to Connect to AP: Error Code: %d. Verifying no pending OTA commits then re-attempting", ret);
        conn_Failure = 1;

    }

#if OTA_SUPPORT
    HTTPSRV_IF_params_t *pHttpSrvParams = NULL;
#if LOCAL_OTA_SUPPORT
    HTTPSRV_IF_params_t httpsSrvParams;
    httpsSrvParams.pClientRootCa = NULL;
    httpsSrvParams.pServerCert = "dummy-root-ca-cert";
    httpsSrvParams.pServerKey = "dummy-root-ca-cert-key";
    httpsSrvParams.primaryPort = 443;
    httpsSrvParams.secondaryPort = 80;
    pHttpSrvParams = &httpsSrvParams;
#endif
    ret = OTA_IF_init(pHttpSrvParams, OtaCallback, 0, NULL);
    if(ret < 0){
        LOG_INFO("failed to init OTA_IF");
        while(1);
    }
#endif

    /* Loop attempt to establish AP Connection */
    while(conn_Failure != 0)
    {
        conn_Failure = SlNetConn_start(SLNETCONN_SERVICE_LVL_INTERNET, SlNetConnEventHandler, SLNETCONN_TIMEOUT, 0);
        LOG_INFO("failed to Connect to AP: Error Code: %d. Retrying...",conn_Failure);
    }

#ifdef MQTT_SECURE_CLIENT

    SetClientIdNamefromMacAddress();
    startSNTP();
    setTime();

#endif

    GPIO_write(CONFIG_GPIO_LED_GREEN_GPIO11, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_LED_YELLOW_GPIO10, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_RED_GPIO09, CONFIG_GPIO_LED_OFF);


/* AP Connection Success, continue MQTT Application */

MQTT_DEMO:

    ret = MQTT_IF_Init(mqttInitParams);
    if(ret < 0){
        while(1);
    }

    /*
     * In case a persistent session is being used, subscribe is called before connect so that the module
     * is aware of the topic callbacks the user is using. This is important because if the broker is holding
     * messages for the client, after CONNACK the client may receive the messages before the module is aware
     * of the topic callbacks. The user may still call subscribe after connect but have to be aware of this.
     */
    ret = MQTT_IF_Subscribe(mqttClientHandle, "topic/awsMainBroker", MQTT_QOS_1, awsMainBroker );
    // ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/otaUpdate", MQTT_QOS_1, startOTA );
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/greenLed", MQTT_QOS_0, toggleGreenLedCallback );
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/yellowLed", MQTT_QOS_0, toggleYellowLedCallback );
    ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/pushDataToAws", MQTT_QOS_1, pushDataToAws);
    // ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/redLed", MQTT_QOS_0, toggleRedLedCallback );
    // ret |= MQTT_IF_Subscribe(mqttClientHandle, "topic/collarData", MQTT_QOS_0, publishSimDataCallback);

    if(ret < 0){
        while(1);
    } else{
        LOG_INFO("Subscribed to all topics successfully\r\n");
    }

    do {
        ret = SlNetConn_waitForConnection(SLNETCONN_SERVICE_LVL_INTERNET, SLNETCONN_TIMEOUT);
    } while(ret != 0);

    LOG_INFO("Wi-Fi connection is UP. Waiting two seconds before MQTT_IF_Connect attempt...");

    sleep(2);

    mqttClientHandle = MQTT_IF_Connect(mqttClientParams, mqttConnParams, MQTT_EventCallback);

    if ((int)mqttClientHandle < 0) {

        LOG_ERROR("MQTT_IF_Connect Error (%d)\r\n", mqttClientHandle);

    } else {

        int pubRet = 0;

        // wait for CONNACK
        while(connected == 0);

        sleep(2);

        LOG_INFO("MQTT connection is UP\n\r");
        LOG_INFO("MQTT_IF_Connect appears to have succeeded. mqttClientHandle = %d\r\n", (int)mqttClientHandle);

        // Try to subscribe now that CONNACK should've happened...

        // Try a test publish to topic/awsMainBroker

        if ((int)mqttClientHandle > 0) {
            pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/awsMainBroker", "FaunaCollar 00000209 says hello!\r\n", strlen("FaunaCollar 00000209 says hello!\r\n"), MQTT_QOS_1);
        }

        if (pubRet < 0) {
            LOG_ERROR("MQTT_IF_Publish to '/topic/awsMainBroker failed, code: %d\r\n", pubRet);
        } else {
            LOG_INFO("MQTT_IF_Publish to '/topic/awsMainBroker succeeded, pubRet: %d\r\n", pubRet);
        }

        sleep(1);

        // Try a test publish to topic/otaUpdate

        // pubRet = 0;
        // if ((int)mqttClientHandle > 0) {
        //    pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/otaUpdate", "otaUpdate listener is ready.\r\n", strlen("otaUpdate listener is ready.\r\n"), MQTT_QOS_1);
        // }

        // if (pubRet < 0) {
        //    LOG_ERROR("MQTT_IF_Publish to '/topic/otaUpdate failed, code: %d\r\n", pubRet);
        // } else {
        //    LOG_INFO("MQTT_IF_Publish to '/topic/otaUpdate succeeded, pubRet: %d\r\n", pubRet);
        // }

        sleep(1);

        // Try a test publish to /topic/yellowLed
//
//        pubRet = 0;
//        if ((int)mqttClientHandle > 0) {
//            pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/yellowLed", "yellowLed=on\r\n", strlen("yellowLed=on\r\n"), MQTT_QOS_0);
//        }
//        if (pubRet < 0) {
//            LOG_ERROR("MQTT_IF_Publish to '/topic/yellowLed failed, code: %d\r\n", pubRet);
//        } else {
//            LOG_INFO("MQTT_IF_Publish to '/topic/yellowLed succeeded, pubRet: %d\r\n", pubRet);
//        }

        // Try a test publish to /topic/collarData

//        pubRet = 0;
//        if ((int)mqttClientHandle > 0) {
//            pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", "collarData!\r\n", strlen("collarData=on\r\n"), MQTT_QOS_1);
//        }
//        if (pubRet < 0) {
//            LOG_ERROR("MQTT_IF_Publish to '/topic/collarData failed, code: %d\r\n", pubRet);
//        } else {
//            LOG_INFO("MQTT_IF_Publish to '/topic/collarData succeeded, pubRet: %d\r\n", pubRet);
//        }

        GPIO_enableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);



        while(1){

            mq_receive(appQueue, (char*)&queueElement, sizeof(struct msgQueue), NULL);

//            if(queueElement.event == APP_MQTT_PUBLISH){
            if(queueElement.event == APP_SEND_DATA_TO_AWS_TRIGGER){
                LOG_INFO("APP_MQTT_PUBLISH\r\n");

                memset(&simData, 0, sizeof(simData));
                // Build some simulated data

                memcpy(&simData.magic, &magic, 10);
                memcpy(&simData.macAddr, &ClientId, 6);
                memcpy(&simData.startBlock, &startBlock, 8);
                memcpy(&simData.endData, &endData, 8);
                memcpy(&simData.endBlock, &endBlock, 8);

                simData.blockNumber = 1;

                // Populate data with random numbers, ints for optics, floats for kinematics

                srand((unsigned) time(&t));

                for (int i=0; i<250; i++) {

                    simData.led1[i]     = rand() % 2800000;
                    simData.led2[i]     = rand() % 1400000;
                    simData.led3[i]     = rand() % 700000;
                    simData.led4[i]     = rand() % 350000;

//                    simData.ax[i]       = sinf(((float)rand() / (float)(280000.5)) * a);
//                    simData.ay[i]       = sinf(((float)rand() / (float)(280000.5)) * a);
//                    simData.az[i]       = sinf(((float)rand() / (float)(280000.5)) * a);
//                    simData.gx[i]       = sinf(((float)rand() / (float)(280000.5)) * a);
//                    simData.gy[i]       = sinf(((float)rand() / (float)(280000.5)) * a);
//                    simData.gz[i]       = sinf(((float)rand() / (float)(280000.5)) * a);

                    simData.ax[i]       = (int16_t)rand() % 150;
                    simData.ay[i]       = (int16_t)rand() % 120;
                    simData.az[i]       = (int16_t)rand() % 100;
                    simData.gx[i]       = (int16_t)rand() % 200;
                    simData.gy[i]       = (int16_t)rand() % 220;
                    simData.gz[i]       = (int16_t)rand() % 250;

                    simData.pitch[i]    = sinf(((float)rand() / (float)(280000.5)) * a);
                    simData.roll[i]     = sinf(((float)rand() / (float)(280000.5)) * a);
                    simData.heading[i]  = sinf(((float)rand() / (float)(280000.5)) * a);

                }

                t = time(NULL);
                simData.timeStamp = t;

                size_t sizeSimData = 0;
                sizeSimData = sizeof(simData);

                UART_PRINT("simData data :: size=%d\n\r", sizeSimData);
                pubRet = 0;
                pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", &simData, sizeSimData, MQTT_QOS_1);

                GPIO_clearInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
                GPIO_enableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);

            } else if(queueElement.event == APP_MQTT_CON_TOGGLE) {

                LOG_TRACE("APP_MQTT_CON_TOGGLE %d\r\n", connected);

                if(connected){
                    ret = MQTT_IF_Disconnect(mqttClientHandle);
                    if(ret >= 0){
                        connected = 0;
                    }
                } else {
                    mqttClientHandle = MQTT_IF_Connect(mqttClientParams, mqttConnParams, MQTT_EventCallback);
                    if((int)mqttClientHandle >= 0) {
                        connected = 1;
                    }
                    /* If failed to re-connect to mqtt start over (this will also include waiting for
                     * the wi-fi connection (in case failure of AP connection caused the disconnection )
                     *  */
                    if(connected == 0)
                        break;
                }
            }
            else if(queueElement.event == APP_MQTT_DEINIT){
                break;
            }
            else if(queueElement.event == APP_OTA_TRIGGER){
                startOTA(NULL, NULL, 0);
            }
            else if(queueElement.event == APP_MQTT_PUBLISH){
            // else if(queueElement.event == APP_SEND_DATA_TO_AWS_TRIGGER){

                GPIO_disableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);

                uint16_t    numSecondsSimData           = 3600;
                uint16_t    numSecondsAggregatedData    = 5;
                uint16_t    numIterationsRequired       = numSecondsSimData / numSecondsAggregatedData;
                                                          /* e.g., 3600 seconds of data, in 5 second increments = 720 loop iterations */

                struct sensorData   simData;

                t = time(NULL);
                simData.timeStamp = t;

                uint32_t    blockNumberIncrement        = 100;
                uint32_t    totalPublishErrors          = 0;
                uint32_t    totalRepublishRetryAttempts = 0;

                for (uint32_t g = 0; g < numIterationsRequired; g++){

                    // Build some simulated data

                    memcpy(&simData.magic, &magic, 10);
                    memcpy(&simData.macAddr, &ClientId, 6);
                    memcpy(&simData.startBlock, &startBlock, 8);
                    memcpy(&simData.endData, &endData, 8);
                    memcpy(&simData.endBlock, &endBlock, 8);

                    simData.blockNumber = g;

                    UART_PRINT("iteration / block number: %d\n\r", g);

                    // Populate data with random numbers, ints for optics, floats for kinematics

                    srand((unsigned) time(&t));

                    for (uint8_t i = 0; i < 250; i++) {

                        simData.led1[i]     = rand() % 2800000;
                        simData.led2[i]     = rand() % 1400000;
                        simData.led3[i]     = rand() % 700000;
                        simData.led4[i]     = rand() % 350000;

                        simData.ax[i]       = (int16_t)rand() % 250;
                        simData.ay[i]       = (int16_t)rand() % 200;
                        simData.az[i]       = (int16_t)rand() % 150;
                        simData.gx[i]       = (int16_t)rand() % 150;
                        simData.gy[i]       = (int16_t)rand() % 200;
                        simData.gz[i]       = (int16_t)rand() % 250;

                        simData.pitch[i]    = sinf(((float)rand() / (float)(280000.5)) * a);
                        simData.roll[i]     = sinf(((float)rand() / (float)(280000.5)) * a);
                        simData.heading[i]  = sinf(((float)rand() / (float)(280000.5)) * a);

                    }   /* end for-loop for creation of simData (i) */

                    t = t + 5;
                    simData.timeStamp   = t;

                    size_t sizeSimData = 0;
                    sizeSimData = sizeof(simData);

                    UART_PRINT("simData data :: size=%d\n\r", sizeSimData);

#ifdef USE_HEATSHRINK
                    uint32_t orig_size = sizeSimData; //strlen(test_data);
                    uint32_t comp_size   = BUFFER_SIZE; //this will get updated by reference
                    uint32_t decomp_size = BUFFER_SIZE; //this will get updated by reference
                    memcpy(orig_buffer, &simData, orig_size);

                    compress(orig_buffer, orig_size, comp_buffer, comp_size);

                    UART_PRINT("compressedData size = %d\n\r", comp_size);

                    pubRet = 0;
                    pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", &comp_buffer, comp_size, MQTT_QOS_1);


#endif



#ifdef USE_LZ4

                    size_t sizeHashMemory       = 10056;
                    size_t sizeCompressedData   = 8192;

                    uint8_t *hashMemory;
                    uint8_t *compressedData;
                    uint8_t *pubData;

                    hashMemory = (uint8_t *)malloc(sizeHashMemory);
                    if (hashMemory != NULL) {
                        UART_PRINT("hashMemory successfully allocated with %d bytes.\n\r", sizeHashMemory);
                    } else {
                        UART_PRINT("hashMemory was not successfully allocated.\n\r");
                    }

                    compressedData = (uint8_t *)malloc(sizeCompressedData);
                    if (compressedData != NULL) {
                        UART_PRINT("compressedData successfully allocated with %d bytes.\n\r", sizeCompressedData);
                    } else {
                        UART_PRINT("compressedData was not successfully allocated.\n\r");
                    }

                    // uint8_t decompressedData[10056]          = {0};

                    size_t      sizeCompressThis;     // sizeDecompressedData,

                    bool compressionSuccessful = FALSE;

                    // sizeHashMemory          = sizeof(hashMemory);
                    // sizeCompressedData      = sizeof(compressedData);
                    // sizeDecompressedData    = sizeof(decompressedData);

                    sizeCompressThis        = sizeSimData;

                    uint32_t contentSize = 0;
                    uint32_t compressedSize = 0;
                    // uint32_t decompressedSize;

                    LZ4_status              status;
                    LZ4_compressParams      compressParams;
                    // LZ4_decompressParams    decompressParams;

                    // memcpy(hashMemory, 0, sizeHashMemory );
                    // memcpy(compressedData, 0, sizeCompressedData);
                    // memset(decompressedData, 0, sizeof(decompressedData));

                    compressParams.src = (void *)&simData;
                    compressParams.dst = (void *)compressedData;
                    // compressParams.dst = compressedData;
                    compressParams.length = sizeCompressThis;
                    compressParams.hashTable = (void *)hashMemory;
                    // compressParams.hashTable = hashMemory;
                    compressParams.hashLog2Size = 12;
                    compressParams.addBlockChecksum = false;
                    compressParams.addContentChecksum = true;
                    compressParams.addContentSize = true;
                    compressedSize = LZ4_compress(&compressParams, &status);

                    if (status == LZ4_SUCCESS) {
                        UART_PRINT("LZ4 compression success, code: %d. Compressed size: %d\n\r", status, compressedSize);
                    } else {
                        UART_PRINT("LZ4 compression error, status code: %d\n\r", &status);
                    }

                    // Read and verify the content size matches the original file size.
                    contentSize = LZ4_getContentSize((void *)compressedData, &status);

                    UART_PRINT("compressData contentSize: %d\n\r", contentSize);
                    if (contentSize == sizeCompressThis) {
                        compressionSuccessful = TRUE;
                        UART_PRINT("compression ratio: %d / %d\n\r", compressedSize, contentSize);
                    } else {
                        compressionSuccessful = FALSE;
                        UART_PRINT("compressed and original size check failed!\n\r");
                    }

        //            // Initialize decompression parameters and decompress data block
        //            decompressParams.src = (void *)compressedData;
        //            decompressParams.dst = (void *)decompressedData;
        //            decompressParams.dstLength = sizeof(decompressedData);
        //            decompressParams.verifyBlockChecksum = true;
        //            decompressParams.verifyContentChecksum = true;
        //            decompressedSize = LZ4_decompress(&decompressParams, &status);
        //
        //            // Verify that the decompressed size is identical to the original file size.
        //            if (decompressedSize != sizeWriteBlockData) {
        //             while(1);
        //            }
        //
        //            // Check the the decompressed data matches the original file.
        //            // if (memcmp((void *)&(onAnimalMail.onAnimalDataPacket->onAnimalDataSamples), (void *)decompressedData, sizeOnAnimalDataPacket)) {    // DKH 20 May 2020 - changed to next line
        //            if (memcmp((void *)&(onAnimalMail.onAnimalData_v1_10), (void *)decompressedData, sizeWriteBlockData)) {
        //             // while(1);
        //                UART_PRINT("tagOS_SD_HAL.cpp :: onAnimalPacketWriter() --> decompressedData != onAnimalMail.onAnimalDataPacket.\n\r");
        //                bool okayToWrite = FALSE;
        //            } else {
        //                UART_PRINT("Compression ratio: %d/%d\n\r", compressedSize, sizeWriteBlockData);
        //                bool okayToWrite = TRUE;
        //                writeBlock.writeBlockHeader.dataSize = sizeWriteBlockData;
        //                writeBlock.writeBlockHeader.usedSize = compressedSize;
        //            }

#ifdef USE_CRC32
                    if (compressionSuccessful == TRUE) {

                        writeBlock.writeBlockHeader.dataSize    = (uint16_t)sizeCompressThis;
                        writeBlock.writeBlockHeader.usedSize    = (uint16_t)compressedSize;
                        writeBlock.writeBlockFooter.blockNumber++;

                        /* Do CRC32_IEEE on compressed data */
                        writeBlock.writeBlockFooter.crc32_ieee = getCrc32_IEEE( (unsigned char *)compressedData, (uint32_t)sizeCompressedData );

                        UART_PRINT("tagOS_SD_HAL :: onAnimalPacketWriter() --> compression successful. Proceed with write...\n\r");

                        memset(writeBlock.writeBlockData, 0, sizeWriteBlockData);
                        memcpy(&writeBlock.writeBlockData, &compressedData, sizeCompressedData );

                    }
#endif

                    pubRet = 0;
                    pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", compressedData, sizeCompressedData, MQTT_QOS_1);

                     if (pubRet == 0) {
                         // free(hashMemory);
                         // free(compressedData);
                     }



#endif  /* #endif for USE_LZ4 */

#ifndef USE_HEATSHRINK
#ifndef USE_LZ4

                    pubRet = 0;
                    pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", &simData, sizeSimData, MQTT_QOS_1);
#endif
#endif
                    if (pubRet < 0) {

                        UART_PRINT("publish failed, iteration = %d, pubRet = %d\n\r", g, pubRet);
                        totalPublishErrors++;

                        if (pubRet == -5) {                     /* no free buffers, try again after timeout */

                            uint8_t retryAttempts = 0;
                            pubFail_minus5++;

                            do {


                                retryStatus = BackoffAlgorithm_GetNextBackoff( &retryParams, rand(), &nextRetryBackoff );
                                retryAttempts++;
                                totalRepublishRetryAttempts++;

                                ( void ) usleep( nextRetryBackoff * 1000U );

                                pubRet = 0;
                                pubRet = MQTT_IF_Publish(mqttClientHandle, "topic/collarData", &simData, sizeSimData, MQTT_QOS_1);

                                if (pubRet == 0) {
                                    republishSuccessOnFailure++;
                                    UART_PRINT("Republish success on retry attempt %d for iteration/block %d.\n\r", retryAttempts, g);
                                } else {
                                    UART_PRINT("Republish failure attempt %d for iteration/block %d\n\r", retryAttempts, g);
                                }

                            } while  ( ( pubRet != 0 ) && ( retryStatus != BackoffAlgorithmRetriesExhausted ) );

                        }

                    }

                }   /* end for-loop for numIterationsRequired (g) */

                UART_PRINT("total publish errors (all types): %d\n\r", totalPublishErrors);
                UART_PRINT("total publish errors (-5; not enough free buffers): %d\n\r", pubFail_minus5);
                UART_PRINT("total successful republishes of publish errors (-5): %d\n\r", republishSuccessOnFailure);
                UART_PRINT("total number of republish attempts after -5 errors: %d\n\r", totalRepublishRetryAttempts);

                GPIO_write(CONFIG_GPIO_LED_YELLOW_GPIO10, 0);

                sleep(5);

                GPIO_clearInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);
                GPIO_enableInt(CONFIG_GPIO_BUTTON_SW3_RIGHT);

            }
            else if(queueElement.event == APP_BTN_HANDLER){

                struct msgQueue queueElement;

                ret = detectLongPress();
                if(ret == 0){

                    LOG_TRACE("APP_BTN_HANDLER SHORT PRESS\r\n");
                    queueElement.event = APP_MQTT_CON_TOGGLE;
                }
                else{

                    LOG_TRACE("APP_BTN_HANDLER LONG PRESS\r\n");
                    queueElement.event = APP_MQTT_DEINIT;
                }

                ret = mq_send(appQueue, (const char*)&queueElement, sizeof(struct msgQueue), 0);
                if(ret < 0){
                    LOG_ERROR("msg queue send error %d", ret);
                }
            }
        }
    }

    deinit = 1;
    if(connected){
        MQTT_IF_Disconnect(mqttClientHandle);
    }
    MQTT_IF_Deinit();

#if OTA_SUPPORT
    if(gNewImageLoaded)
    {
        SlNetConn_stop(SlNetConnEventHandler);
        OTA_IF_install();
    }
#endif

    LOG_INFO("looping the MQTT functionality of the example for demonstration purposes only\r\n");
    sleep(1);
    goto MQTT_DEMO;

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
