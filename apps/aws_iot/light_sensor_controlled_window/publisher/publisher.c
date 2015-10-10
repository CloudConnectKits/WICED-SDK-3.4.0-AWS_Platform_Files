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
 * Light sensor controlled window publisher application
 *
 * This application reads room light intensity every 5 seconds.
 * And publishes "ON" or "OFF" based on the difference in light intensity.
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
#include "aws_common.h"
/******************************************************
 *                      Macros
 ******************************************************/
#define CLIENT_ID                           "wiced_publisher_aws"
#define BUFFER_LENGTH                       (2048)
#define MQTT_PUBLISH_RETRY_COUNT            (3)
#define MSG_ON                              "ON"
#define MSG_OFF                             "OFF"
#define LIGHT_THRESHOLD                     200

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static aws_app_info_t  app_info =
{
    .mqtt_client_id = CLIENT_ID
};


/******************************************************
 *               Static Function Definitions
 ******************************************************/
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
    char            *msg = MSG_OFF;
    wiced_result_t   ret = WICED_SUCCESS;
    int              retries;
    uint16_t         light_value;

    ret = aws_app_init(&app_info);

    /* Initialise Light sensor */
    WPRINT_APP_INFO( ("Initializing Light Sensor\n" ));
    wiced_adc_init( WICED_LIGHT, 5 );

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
            /* Read light sensor */
            wiced_adc_take_sample( WICED_LIGHT, &light_value );
            WPRINT_APP_INFO( ("Light value %u\n", light_value) );

            if ( light_value >= LIGHT_THRESHOLD )
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
                retries++ ;
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
