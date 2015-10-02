/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * Temperature sensor controlled window publisher application
 *
 * This application reads room temperature every 5 seconds and gets outside temperature from www.openweathermap.org.
 * And publishes "LIGHT ON" or "LIGHT OFF" based on the difference in temperature with QOS-1.
 *
 * If temperature difference is >= 5 degrees publish "LIGHT ON" to topic "WICED_BULB"
 * else publish "LIGHT OFF" to topic "WICED_BULB"
 *
 * To demonstrate the app, work through the following steps.
 * 1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the wifi_config_dct.h header file to match your Wi-Fi access point
 * 2. Modify the BROKER HOST NAME to have the ip address of your MQTT broker.
 * 3. Copy required certificates( rootCA.pem, cert.pem, privkey.pem ) in resources/apps/secure_mqtt folder.
 * 3. Plug the WICED eval board into your computer
 * 4. Open a terminal application and connect to the WICED eval board
 * 5. Build and download the application (to the WICED board)
 * Tested on BCM43364WCD1 as of now.
 *
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include <math.h>
#include "thermistor.h" // Using Murata NCP18XH103J03RB thermistor
#include "http.h"
#include "cJSON.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MQTT_BROKER_ADDRESS                 "data.iot.us-east-1.amazonaws.com"
#define WICED_TOPIC                         "WICED_BULB"
#define CLIENT_ID                           "wiced_publisher_aws"
#define MQTT_REQUEST_TIMEOUT                (5000)
#define MQTT_DELAY_IN_MILLISECONDS          (1000)
#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)
#define BUFFER_LENGTH                       (2048)
#define MQTT_PUBLISH_RETRY_COUNT            (3)
#define WEATHER_HTTP_GET_REQUEST            "GET /data/2.5/weather?q=bangalore HTTP/1.1\r\n" \
                                            "Host: openweathermap.org\r\n" \
                                            "Connection: close\r\n" \
                                            "\r\n"
#define MSG_ON                              "ON"
#define MSG_OFF                             "OFF"

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    int16_t               last_sample;
} temp_data_t;

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_ip_address_t                   broker_address;
static wiced_mqtt_event_type_t              expected_event;
static wiced_semaphore_t                    semaphore;
static wiced_mqtt_security_t                security;
static temp_data_t                          temperature_data;
uint8_t                                     data[10];
wiced_ip_address_t                          http_address;
uint8_t                                     buffer[BUFFER_LENGTH];

/******************************************************
 *               Static Function Definitions
 ******************************************************/
static void get_weather_info( float *value)
{
    cJSON *json, *original;
    char *out;
    wiced_result_t result;
    result = wiced_http_get( &http_address, WEATHER_HTTP_GET_REQUEST, buffer, sizeof(buffer) );
    if ( result == WICED_SUCCESS )
    {
        original = json = cJSON_Parse( strstr( (char*) buffer, "{\"coord\"" ) );
        if ( json != NULL )
        {
            json = cJSON_GetArrayItem( json, 3 );
            json = cJSON_GetArrayItem( json, 0 );
            out = cJSON_Print( json );
            cJSON_Delete( original );
            *value = atof( out );
        }
    }
    else
        WPRINT_APP_INFO( ( "Get failed: %u\n", result ) );
}

/*
 * A blocking call to an expected event.
 */
static wiced_result_t mqtt_wait_for( wiced_mqtt_event_type_t event, uint32_t timeout )
{
    if ( wiced_rtos_get_semaphore( &semaphore, timeout ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    else
    {
        if ( event != expected_event )
        {
            return WICED_ERROR;
        }
    }
    return WICED_SUCCESS;
}

/*
 * Open a connection and wait for MQTT_REQUEST_TIMEOUT period to receive a connection open OK event
 */
static wiced_result_t mqtt_conn_open( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security )
{
    wiced_mqtt_pkt_connect_t conninfo;
    wiced_result_t ret = WICED_SUCCESS;
    memset( &conninfo, 0, sizeof( conninfo ) );
    conninfo.port_number = 0;
    conninfo.mqtt_version = WICED_MQTT_PROTOCOL_VER4;
    conninfo.clean_session = 1;
    conninfo.client_id = (uint8_t*) CLIENT_ID;
    conninfo.keep_alive = 5;
    conninfo.password = NULL;
    conninfo.username = NULL;
    ret = wiced_mqtt_connect( mqtt_obj, address, interface, callback, security, &conninfo );
    if ( ret != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Publish (send) message to WICED_TOPIC and wait for 5 seconds to receive a PUBCOMP (as it is QoS=2).
 */
static wiced_result_t mqtt_app_publish( wiced_mqtt_object_t mqtt_obj, uint8_t qos, uint8_t *topic, uint8_t *data, uint32_t data_len )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_publish( mqtt_obj, topic, data, data_len, qos );

    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }

    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_PUBLISHED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}
/*
 * Close a connection and wait for 5 seconds to receive a connection close OK event
 */
static wiced_result_t mqtt_conn_close( wiced_mqtt_object_t mqtt_obj )
{
    if ( wiced_mqtt_disconnect( mqtt_obj ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_DISCONNECTED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Call back function to handle MQTT events
 */
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    switch ( event->type )
    {

        case WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS:
        case WICED_MQTT_EVENT_TYPE_DISCONNECTED:
        case WICED_MQTT_EVENT_TYPE_PUBLISHED:
        case WICED_MQTT_EVENT_TYPE_SUBCRIBED:
        case WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED:
        {
            expected_event = event->type;
            wiced_rtos_set_semaphore( &semaphore );
        }
            break;
        case WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED:
        default:
            break;
    }
    return WICED_SUCCESS;
}

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Main application
 */
void application_start( void )
{
    wiced_mqtt_object_t mqtt_object;
    wiced_result_t ret = WICED_SUCCESS;
    uint32_t size_out;
    float value = 0, delta = 0;
    int connection_retries = 0;
    int retries = 0;
    char *msg = MSG_OFF;
    wiced_init( );

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* http setup */
    ret = wiced_hostname_lookup( "openweathermap.org", &http_address, 10000 );
    if ( ret == WICED_ERROR )
    {
        WPRINT_APP_INFO(("Error in resolving DNS\n"));
        return;
    }
    WPRINT_APP_INFO( ( "http Server is at %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 0) ) );

    /* Initialise Thermistor */
    wiced_adc_init( WICED_THERMISTOR_JOINS_ADC, 5 );
    memset( &temperature_data, 0, sizeof( temperature_data ) );

#ifdef USE_AWS_MQTT_CERTIFICATES
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_rootCA_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.ca_cert );
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_privkey_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.key );
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_cert_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.cert );
#endif
    /* Mqtt Setup */
    /* Memory allocated for mqtt object*/
    mqtt_object = (wiced_mqtt_object_t) malloc( WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if ( mqtt_object == NULL )
    {
        WPRINT_APP_ERROR("Dont have memory to allocate for mqtt object...\n");
        return;
    }

    WPRINT_APP_INFO( ( "Resolving IP address of MQTT broker\n" ) );
    ret = wiced_hostname_lookup( MQTT_BROKER_ADDRESS, &broker_address, 10000 );
    if ( ret == WICED_ERROR )
    {
        WPRINT_APP_INFO(("Error in resolving DNS\n"));
        return;
    }
    WPRINT_APP_INFO(("[MQTT] Connecting to broker %u.%u.%u.%u ...\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));

    wiced_mqtt_init( mqtt_object );
    wiced_rtos_init_semaphore( &semaphore );

    do
    {
        WPRINT_APP_INFO(("[MQTT] Opening connection..."));
        do
        {
            ret = mqtt_conn_open( mqtt_object, &broker_address, WICED_STA_INTERFACE, mqtt_connection_event_cb, &security );
            connection_retries++ ;
        } while ( ( ret != WICED_SUCCESS ) && ( connection_retries < WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES ) );

        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Failed\n"));
            break;
        }
        WPRINT_APP_INFO(("Success\n"));

        while ( 1 )
        {
            /* Getting the temperature value for bangalore location from www.openweathermap.org */
            WPRINT_APP_INFO(( "Getting weather info\n" ));
            get_weather_info( &value );

            /* Reading temperature sensor value */
            thermistor_take_sample( WICED_THERMISTOR_JOINS_ADC, &temperature_data.last_sample );
            delta = temperature_data.last_sample - value;
            WPRINT_APP_INFO(( " delta  %f\n", (delta/10) ));

            if ( abs( delta )  >= 50)
            {
                msg = MSG_ON;
            }
            else
            {
                msg = MSG_OFF;
            }
            /* Controlling the LED by publishing to mqtt topic "WICED_BULB" */
            WPRINT_APP_INFO(("[MQTT] Publishing..."));
            do
            {
                ret = mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) WICED_TOPIC, (uint8_t*) msg, strlen( msg ) );
                retries++ ;
            } while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_PUBLISH_RETRY_COUNT ) );
            if ( ret != WICED_SUCCESS )
            {
                WPRINT_APP_INFO((" Failed\n"));
                break;
            }
            else
            {
                WPRINT_APP_INFO((" Success\n"));
            }

            wiced_rtos_delay_milliseconds( 5000 );
        }

        WPRINT_APP_INFO(("[MQTT] Closing connection..."));
        mqtt_conn_close( mqtt_object );

        wiced_rtos_delay_milliseconds( MQTT_DELAY_IN_MILLISECONDS * 2 );
    } while ( 1 );

    wiced_rtos_deinit_semaphore( &semaphore );
    WPRINT_APP_INFO(("[MQTT] Deinit connection...\n"));
    ret = wiced_mqtt_deinit( mqtt_object );
    free( mqtt_object );
    mqtt_object = NULL;

    return;
}
