/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * Sensor controlled window subscriber application
 *
 * This application subscribes to topic "WICED_BULB" with Qos0
 * And toggles the LED1 on BCM943364WCD1 based on message received.
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Modify the BROKER HOST NAME to have the ip address of your MQTT broker.
 *  3. Copy required certificates( rootCA.pem, cert.pem, privkey.pem ) in resources/apps/secure_mqtt folder.
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include "cJSON.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MQTT_BROKER_ADDRESS                 "data.iot.us-east-1.amazonaws.com"
#define WICED_TOPIC                         "WICED_BULB"
#define THING_STATE_TOPIC                   "$shadow/beta/state/AWS"
#define THING_UPDATED_TOPIC                 "$shadow/beta/sync/AWS"
#define SHADOW_PUBLISH_MESSAGE_STR_OFF      "{ \"state\": {\"reported\": { \"LIGHT\": \"OFF\" } } }"
#define SHADOW_PUBLISH_MESSAGE_STR_ON       "{ \"state\": {\"reported\": { \"LIGHT\": \"ON\" } } }"
#define CLIENT_ID                           "wiced_subcriber_aws"
#define MQTT_REQUEST_TIMEOUT                (5000)
#define MQTT_DELAY_IN_MILLISECONDS          (1000)
#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)
#define WICED_MQTT_SUBSCRIBE_RETRY_COUNT    (3)

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_ip_address_t broker_address;
static wiced_mqtt_event_type_t expected_event;
static wiced_semaphore_t semaphore;
static wiced_mqtt_security_t security;
static char* led_status = "OFF";
static wiced_semaphore_t wake_semaphore;

/******************************************************
 *               Static Function Definitions
 ******************************************************/
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
 * Publish (send) WICED_MESSAGE_STR to WICED_TOPIC and wait for 5 seconds to receive a PUBCOMP (as it is QoS=2).
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
 * Call back function to handle connection events.
 */
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    cJSON *json, *original;
    char* out = NULL;

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
        {
            wiced_mqtt_topic_msg_t msg = event->data.pub_recvd;
            WPRINT_APP_INFO(( "[MQTT] Received %.*s  for TOPIC : %.*s\n\n", (int) msg.data_len, msg.data, (int) msg.topic_len, msg.topic ));

            if ( strncmp( (char*) msg.topic, (const char*) WICED_TOPIC, msg.topic_len ) == 0 )
            {
                printf( "Requested WICED_BULB State[%s] Current WICED_BULB State [%s]\n", msg.data, led_status );

                if ( strncasecmp( (char*) msg.data, led_status, msg.data_len ) )
                {
                    wiced_rtos_set_semaphore( &wake_semaphore );
                }
                else
                {
                    break;
                }
            }

            else if ( strncmp( (char*) msg.topic, THING_UPDATED_TOPIC, msg.topic_len ) == 0 )
            {
                original = json = cJSON_Parse( strstr( (char*) msg.data, "{\"version\"" ) );
                if ( json != NULL )
                {
                    json = cJSON_GetArrayItem( json, 1 );
                    json = cJSON_GetArrayItem( json, 0 );
                    out = cJSON_Print( json );
                    cJSON_Delete( original );
                    WPRINT_APP_INFO(( "Requested LED State[%s] Current LED State [%s]\n", out, led_status ));

                    if ( out == NULL || strncasecmp( out, led_status, msg.data_len ) == 0 )
                    {
                        break;
                    }
                    else
                    {
                        wiced_rtos_set_semaphore( &wake_semaphore );
                    }
                }
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
 * Subscribe to WICED_TOPIC and wait for 5 seconds to receive an ACM.
 */
static wiced_result_t mqtt_app_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_subscribe( mqtt_obj, topic, qos );
    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_SUBCRIBED, MQTT_REQUEST_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    static wiced_mqtt_object_t mqtt_object;
    wiced_result_t ret = WICED_SUCCESS;
    wiced_mutex_t mutex;
    int connection_retries = 0;
    int retries = 0;
    uint32_t size_out;

    wiced_init( );
    wiced_rtos_init_mutex( &mutex );

    /* Disable roaming to other access points */
    wiced_wifi_set_roam_trigger( -99 ); /* -99dBm ie. extremely low signal level */

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

#ifdef USE_AWS_MQTT_CERTIFICATES
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_rootCA_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.ca_cert );
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_privkey_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.key );
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_cert_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.cert );
#endif
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
    wiced_rtos_init_semaphore( &wake_semaphore );

    WPRINT_APP_INFO(("[MQTT] Opening connection..."));
    do
    {
        ret = mqtt_conn_open( mqtt_object, &broker_address, WICED_STA_INTERFACE, mqtt_connection_event_cb, &security );
        connection_retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( connection_retries < WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO((" Failed\n"));
        return;
    }
    else
    {
        WPRINT_APP_INFO((" Success\n"));

    }

    WPRINT_APP_INFO(("[MQTT] Publishing...\n"));
    mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) THING_STATE_TOPIC, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF, sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF ) );

    wiced_rtos_delay_milliseconds( MQTT_DELAY_IN_MILLISECONDS * 2 );

    WPRINT_APP_INFO(("[MQTT] Subscribing...\n"));
    mqtt_app_subscribe( mqtt_object, THING_UPDATED_TOPIC, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE );

    WPRINT_APP_INFO(("[MQTT] Subscribing...\n"));
    do
    {
        ret = mqtt_app_subscribe( mqtt_object, WICED_TOPIC, WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE );
        retries++ ;
    } while ( ( ret != WICED_SUCCESS ) && ( retries < WICED_MQTT_SUBSCRIBE_RETRY_COUNT ) );

    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO((" Failed\n"));
        return;
    }

    WPRINT_APP_INFO(("Success...\n"));

    while ( 1 )
    {
        /* Wait forever on wake semaphore until the LED status is changed */
        wiced_rtos_get_semaphore( &wake_semaphore, WICED_NEVER_TIMEOUT );

        /* Toggle the LED */
        if ( strncasecmp( led_status, "OFF", 3 ) == 0 )
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            WPRINT_APP_INFO(("[MQTT] Publishing..."));
            mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) THING_STATE_TOPIC, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_ON, sizeof( SHADOW_PUBLISH_MESSAGE_STR_OFF ) );
        }
        else
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            WPRINT_APP_INFO(("[MQTT] Publishing..."));
            mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*) THING_STATE_TOPIC, (uint8_t*) SHADOW_PUBLISH_MESSAGE_STR_OFF, sizeof( SHADOW_PUBLISH_MESSAGE_STR_ON ) );
        }
    }

    WPRINT_APP_INFO(("[MQTT] Closing connection..."));
    mqtt_conn_close( mqtt_object );

    wiced_rtos_deinit_semaphore( &semaphore );
    WPRINT_APP_INFO(("[MQTT] Deinit connection...\n"));
    ret = wiced_mqtt_deinit( mqtt_object );
    wiced_rtos_deinit_semaphore( &wake_semaphore );
    free( mqtt_object );
    mqtt_object = NULL;

    return;
}
