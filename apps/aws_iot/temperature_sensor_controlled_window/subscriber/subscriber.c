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
 * Sensor smart_controlled window subscriber application
 *
 * This application subscribes to selected topic from webui and subscribes to shadow related topics
 * And toggles the LED1 based on message received.
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
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include "cJSON.h"
#include "aws_common.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED    "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"OFF\" , \"auto\": \"NO\"} ,\
                                                                        \"reported\": { \"status\": \"OFF\" ,\"auto\": \"NO\"} \
                                                                    }\
                                                                }"

#define SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED     "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"ON\" , \"auto\": \"YES\" } ,\
                                                                        \"reported\": { \"status\": \"ON\" ,\"auto\": \"YES\"} \
                                                                    }\
                                                                }"
#define SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON    "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"OFF\" , \"auto\": \"YES\" } ,\
                                                                        \"reported\": { \"status\": \"OFF\" ,\"auto\": \"YES\"} \
                                                                    }\
                                                                }"

#define SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF     "{\
                                                                    \"state\":\
                                                                    {\
                                                                        \"desired\": { \"status\": \"ON\" , \"auto\": \"NO\" },\
                                                                        \"reported\": { \"status\": \"ON\" ,\"auto\": \"NO\"}\
                                                                    }\
                                                                }"

#define CLIENT_ID                           "wiced_subcriber_aws"
#define WICED_MQTT_SUBSCRIBE_RETRY_COUNT    (3)

/******************************************************
 *               Variable Definitions
 ******************************************************/
static aws_app_info_t app_info =
                        {
                        .mqtt_client_id = CLIENT_ID
                        };
static char*          led_status = "OFF";
static uint8_t        smart_control = 0;
static uint8_t        button_pressed = 0;

/******************************************************
 *               Static Function Definitions
 ******************************************************/
/*
 * Call back function to handle connection events.
 */
wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    cJSON *json = NULL;
    cJSON *original = NULL;

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
        {
            cJSON *child = NULL;
            wiced_mqtt_topic_msg_t msg = event->data.pub_recvd;
            WPRINT_APP_DEBUG(( "[MQTT] Received %.*s  for TOPIC : %.*s\n", (int) msg.data_len, msg.data, (int) msg.topic_len, msg.topic ));

            if ( strncmp( (char*) msg.topic, (const char*) app_info.thing_name, msg.topic_len ) == 0 )
            {
                WPRINT_APP_DEBUG (( "Requested WICED_BULB State[%.*s] Current WICED_BULB State [%s]\n", (int) msg.data_len, msg.data, led_status ));

                if ( strncasecmp( (char*) msg.data, led_status, msg.data_len ) )
                {
                    if ( smart_control == 1 )
                    {
                        if ( strncmp( (char*) msg.data, "ON", 2 ) == 0 )
                            led_status = "ON";
                        else
                            led_status = "OFF";

                        wiced_rtos_set_semaphore( &app_info.wake_semaphore );
                    }
                }
                else
                {
                    break;
                }
            }

            else if ( strncmp( (char*) msg.topic, app_info.shadow_delta_topic, msg.topic_len ) == 0 )
            {
                cJSON* temp;

                original = cJSON_Parse( strchr( (char*) msg.data, '{' ) );
                child = original->child;
                json = NULL;

                while ( child != NULL )
                {
                    WPRINT_APP_DEBUG (( "child->string [%s]\n", child->string ));
                    if ( strcmp( child->string, "state" ) == 0 )
                    {
                        json = child;
                        break;
                    }
                    child = child->next;
                }

                if ( json != NULL )
                {

                    temp = cJSON_GetObjectItem( json, "auto" );

                    if ( temp )
                    {

                        if ( strncmp( temp->valuestring, "YES", 3 ) == 0 )
                        {
                            smart_control = button_pressed = 1;
                            WPRINT_APP_INFO (( "smart_control enabled\n" ));
                        }
                        else
                        {
                            smart_control = button_pressed = 0;
                            WPRINT_APP_INFO (( "smart_control not enabled\n" ));
                        }
                    }

                    temp = cJSON_GetObjectItem( json, "status" );

                    if ( temp )
                    {
                        WPRINT_APP_INFO(( "Requested updated state from AWS thing LED State [%s] Current LED State [%s]\n", temp->string, led_status ));

                        if ( strncmp( temp->valuestring, "ON", 2 ) == 0 )
                        {
                            led_status = "ON";
                        }
                        else
                        {
                            led_status = "OFF";
                        }

                        wiced_rtos_set_semaphore( &app_info.wake_semaphore );
                    }
                }
                cJSON_Delete( original );
            }
            else
            {
                WPRINT_APP_INFO(( "Topic Not found\n" ));
            }
        }
            break;
        default:
            break;
    }
    return WICED_SUCCESS;
}

static void publish_callback( void* arg )
{
    if ( button_pressed == 0 )
    {
        smart_control = button_pressed = 1;
    }
    else
    {
        smart_control = button_pressed = 0;
    }

    WPRINT_APP_INFO (( "button_pressed %d\n", button_pressed ));
}
/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    wiced_result_t ret = WICED_SUCCESS;
    uint32_t       connection_retries = 0;
    uint32_t       retries = 0;

    ret = aws_app_init( &app_info );

    wiced_gpio_input_irq_enable( WICED_BUTTON1, IRQ_TRIGGER_RISING_EDGE, publish_callback, NULL );
    do
    {
        ret = aws_mqtt_conn_open( app_info.mqtt_object, mqtt_connection_event_cb );
        connection_retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( connection_retries < WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES ) );

    aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.shadow_state_topic, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED, sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED ) );

    wiced_rtos_delay_milliseconds( MQTT_DELAY_IN_MILLISECONDS * 2 );

    aws_mqtt_app_subscribe( app_info.mqtt_object, app_info.shadow_delta_topic, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE );

    do
    {
        ret = aws_mqtt_app_subscribe( app_info.mqtt_object, app_info.thing_name, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE );
        retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( retries < WICED_MQTT_SUBSCRIBE_RETRY_COUNT ) );
    if ( ret != WICED_SUCCESS )
    {
        return;
    }

    while ( 1 )
    {
        /* Wait forever on wake semaphore until the LED status is changed */
        wiced_rtos_get_semaphore( &app_info.wake_semaphore, WICED_NEVER_TIMEOUT );

        /* Toggle the LED */
        if ( ( strncasecmp( led_status, "OFF", 3 ) == 0 ) && smart_control == 1 )
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            WPRINT_APP_INFO(("[MQTT] Publishing to Thing state topic\n"));
            aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.shadow_state_topic, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON, sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED_STATUS_ON ) );
        }
        else if ( ( strncasecmp( led_status, "ON", 2 ) == 0 ) && smart_control == 0 )
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            WPRINT_APP_INFO(("[MQTT] Publishing to Thing state topic\n"));
            aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.shadow_state_topic, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF, sizeof( SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED_STATUS_OFF ) );
        }
        else if ( ( strncasecmp( led_status, "ON", 2 ) == 0 ) && smart_control == 1 )
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            WPRINT_APP_INFO(("[MQTT] Publishing to Thing state topic\n"));
            aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.shadow_state_topic, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED, sizeof( SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED ) );
        }
        else
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            WPRINT_APP_INFO(("[MQTT] Publishing to Thing state topic\n"));
            aws_mqtt_app_publish( app_info.mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) app_info.shadow_state_topic, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED, sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED ) );
        }
    }

    aws_mqtt_conn_close( app_info.mqtt_object );

    wiced_rtos_deinit_semaphore( &app_info.msg_semaphore );
    ret = wiced_mqtt_deinit( app_info.mqtt_object );
    wiced_rtos_deinit_semaphore( &app_info.wake_semaphore );
    free( app_info.mqtt_object );
    app_info.mqtt_object = NULL;

    return;
}
