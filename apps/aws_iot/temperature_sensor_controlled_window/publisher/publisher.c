/*
 * Copyright 2015, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *
 * Temperature sensor controlled window publisher application
 *
 * This application reads room temperature every 5 seconds and gets outside temperature from www.openweathermap.org.
 * And publishes "LIGHT ON" or "LIGHT OFF" based on the difference in temperature with QOS-1.
 *
 * If temperature difference is >= 5 degrees publish "ON" to topic configured via webui
 * else publish "OFF" to topic configured via webui
 *
 * To demonstrate the app, work through the following steps.
 * 1. Copy required certificates( rootCA.pem ) in resources/apps/secure_mqtt folder.
 * 2. configuring the device :
 *    1) connect to WICED which runs as SoftAP. SSID and credentials are as per DCT configuration.
 *    2) After successful connection WICED device will act as WebServer. with IP address 192.168.0.1.
 *    3) From host system type URL in browser as 192.168.0.1.
 *    4) Configure settings for thing or topic name and upload certificate and private key. and do wifi configurations.
 *    5) WICED will reboot.
 * 3. WICED will connect to selected Wi-Fi configurations. And connect to broker.
 * 4. Plug the WICED eval board into your computer
 * 5. Open a terminal application and connect to the WICED eval board
 * 6. Build and download the application (to the WICED board)
 *
 *
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include <math.h>
#include "thermistor.h" // Using Murata NCP18XH103J03RB thermistor
#include "http.h"
#include "cJSON.h"
#include "aws_common.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define CLIENT_ID                           "wiced_publisher_aws"
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
static aws_app_info_t  app_info =
{
    .mqtt_client_id = CLIENT_ID
};

static temp_data_t        temperature_data;
uint8_t                   data[10];
wiced_ip_address_t        http_address;
uint8_t                   buffer[BUFFER_LENGTH];

/******************************************************
 *               Static Function Definitions
 ******************************************************/
static void get_weather_info( float *value)
{
    int http_status_code = 0;
    cJSON *original, *child;
    wiced_result_t result;
    result = wiced_http_get( &http_address, WEATHER_HTTP_GET_REQUEST, buffer, sizeof( buffer ) );
    if ( result == WICED_SUCCESS )
    {
        http_status_code = atoi((char*)(buffer+(strlen( "HTTP/1.1" ) + 1)));
        if ( http_status_code < 200 && http_status_code > 299 )
        {
            WPRINT_APP_INFO( ( "HTTP error code: %d\n", http_status_code ) );
            return;
        }
        original = cJSON_Parse( strstr( (char*) buffer, "{\"coord\"" ) );
        if ( original != NULL )
        {
            child = original->child;
            while ( child != NULL )
            {
                WPRINT_APP_DEBUG(( "child->string [%s]\n", child->string ));
                if ( strcmp( child->string, "main" ) == 0 )
                {
                    child = cJSON_GetObjectItem( child, "temp" );
                    *value = child->valuedouble;
                    break;
                }
                child = child->next;
            }
            cJSON_Delete( original );
        }
    }
    else
        WPRINT_APP_INFO( ( "Get failed: %u\n", result ) );
}

/*
 * Call back function to handle MQTT events
 */
wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    switch ( event->type )
    {
        case WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS:
        case WICED_MQTT_EVENT_TYPE_DISCONNECTED:
        case WICED_MQTT_EVENT_TYPE_PUBLISHED:
        case WICED_MQTT_EVENT_TYPE_SUBCRIBED:
        case WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED:
        {
            app_info.expected_event = event->type;
            wiced_rtos_set_semaphore( &app_info.msg_semaphore );
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
    float            value = 0, delta = 0;
    char            *msg = MSG_OFF;
    wiced_result_t   ret = WICED_SUCCESS;
    int              retries;

    ret = aws_app_init(&app_info);

    ret = wiced_hostname_lookup( "openweathermap.org", &http_address, 10000 );
    if ( ret == WICED_ERROR )
    {
        WPRINT_APP_INFO(("Error in resolving DNS\n"));
        return;
    }
    WPRINT_APP_INFO( ( "Weather Server IP: %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(http_address) >> 0) ) );

    /* Initialize thermistor */
    wiced_adc_init( WICED_THERMISTOR_JOINS_ADC, 5 );
    memset( &temperature_data, 0, sizeof( temperature_data ) );

    do
    {
        ret = aws_mqtt_conn_open( app_info.mqtt_object, mqtt_connection_event_cb );
        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Failed\n"));
            break;
        }

        while ( 1 )
        {
            /* Getting the temperature value for bangalore location from www.openweathermap.org */
            WPRINT_APP_INFO(( "Getting weather info\n" ));
            get_weather_info( &value );

            /* Reading temperature sensor value */
            thermistor_take_sample( WICED_THERMISTOR_JOINS_ADC, &temperature_data.last_sample );
            delta = temperature_data.last_sample - value;
            WPRINT_APP_INFO(( "delta %f\n", (delta/10) ));

            if ( abs( delta ) >= 50 )
            {
                msg = MSG_ON;
            }
            else
            {
                msg = MSG_OFF;
            }
            /* Controlling the LED by publishing to mqtt topic "WICED_BULB" */
            retries = 0;
            do
            {
                ret = aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.thing_name, (uint8_t*) msg, strlen( msg ) );
                retries++;
            } while ( ( ret != WICED_SUCCESS ) && ( retries < MQTT_PUBLISH_RETRY_COUNT ) );
            if ( ret != WICED_SUCCESS )
            {
                break;
            }

            wiced_rtos_delay_milliseconds( 5000 );
        }

        aws_mqtt_conn_close( app_info.mqtt_object );

        wiced_rtos_delay_milliseconds( MQTT_DELAY_IN_MILLISECONDS * 2 );
    } while ( 1 );

    aws_mqtt_conn_close( app_info.mqtt_object );

    wiced_rtos_deinit_semaphore( &app_info.msg_semaphore );
    WPRINT_APP_INFO(("[MQTT] Deinit connection...\n"));
    ret = wiced_mqtt_deinit( app_info.mqtt_object );
    free( app_info.mqtt_object );
    app_info.mqtt_object = NULL;

    return;
}
