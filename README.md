This is the base standalone MQTT client app for connecting, subscribing, and publishing MQTT messages with AWS IoT Core.

This project uses the following settings:

#define MQTT_CONNECTION_ADDRESS         "_____________-ats.iot.us-east-1.amazonaws.com"

#define MQTT_CONNECTION_PORT_NUMBER     8883

#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_URL | MQTTCLIENT_NETCONN_SEC \
                                        | MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION

char *MQTTClient_secureFiles[4] = { "__________9_privkey.pem", "__________9.pem", "sf-class2-root.pem", NULL };

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

The secure files have a few important conditions that need to be met for this to work correctly:

1. Both the _________XX_privkey.pem and _________XX.pem key, provided by
single Thing provisioning on AWS, are in .pem format. When flashing
these files to Uniflash, the .pem filename extension MUST BE USED.
Ditching the .pem will cause CR+LF to be retained, which will silently
cause certificate and key parsing to fail, resulting in a very
late-stage AWS TLS connection failure.


2. Likewise, the use of the Starfield Root Certification Authority
sf-class2-root.pem, must have the .pem filename extension for correct
parsing. The documented/suggested use of Amazon Root CA 1 or Amazon Root
CA 

3 .pem certificates will not work with the CC32XX device lineup. 3.
The DH public key file does not appear to be needed, and previous
testing seemed to cause this to fail. It may work if the DH public key
is included as with a .pem filename extension, but this works as-is and
can be ignored for now. 

4. Make sure to use the endpoint noted above.
Logging in to AWS directly, without specifying the _ company
account, results in a different AWS account being used when logging in
with my firstname.lastname@googleworkspace.com account. Make sure to use the
_ AWS administrator account only! 

5. It's also necessary to use
a custom version of mqttclient.c, included now in the /ifmod
subdirectory. The default MQTT library by TI has a default max buffer
size of 1024 bytes. There don't appear to be any makefiles for
rebuilding the library for TICLANG used in SL710, but it seems to
compile with updated defaults by including mqttclient.c directly in the
source tree for this app. I've set the buffer size to 8192 for now. Base
simData size is about 2.6 kB per second of data at 50 Hz, minus power
data which won't add much. 

6. It's not clear if / how well this approach
is thread-safe. When porting this functionality to
projectName_CC32XXSF_SL710, assess carefully how this new functionality may
block important events (I2C sample retrieval, SD writes, etc.). 

7. When
porting this functionality to projectName_CC32XXSF_SL710, be mindful that
the later is a merger of mqtt_client_over_tls_1_3 and
tagOS_CC32xx_SL340. Ensure that TLS 1.3 functionality is stripped out
or, even better, that #ifdef logic is added to allow mbedTLS1.3
functionality in the future. 

8. OTA FUNCTIONALITY IS BROKEN! There seems
to be a buffer size problem with the default SL710 OTA library, probably
choking on how big the mcuflashimg.bin was when trying this with mbedTLS
support. The image size is significantly smaller for this project, so
test OTA and see whether it continues to fail during firmware image
upload when porting this to projectName. If needed, figure out where the
OTA buffer size can be tweaked and either include that .c file in the
/ifmod source directory or rebuild the OTA library with the updated
buffer size. 

9. AWS IOT SECURITY IS WIDE OPEN AND SHOULDN'T BE - I had
to use an AWS IOT Core policy of, basically, allowing all connections to
do whatever. This needs to be pared down to allow only connects,
subscribes, and publishes. 

10. AWS IOT FLEET MANAGEMENT WITH JIT
CERTIFICATE ENROLLMENT HAS NOT BEEN IMPLEMENTED. For now I'm just using
single issue certs on a per-Thing basis. This needs to change. See the
fleetProvisioning topic below to restart my thinking on this. 

11. Need
to consider and implement only the necessary topics, i.e., sensorData,
metaData, otaUpdate, fleetProvisioning, deviceConfig. Anything else we
need or want?

I was never able to get the mbedTLS 1.3 version of this to work. Now
that a known configuration works for TLS 1.2 using standard SimpleLink
7.10 NWP ROM routines, it may be worth revisiting mbedTLS 1.3 in the
future.
