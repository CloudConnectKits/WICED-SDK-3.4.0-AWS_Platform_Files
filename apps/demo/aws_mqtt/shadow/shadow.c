/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * AWS Thing Shadow Demo Application
 *
 * This application demonstrates how you can publish state from WICED and get sync with AWS thing shadow.
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * Once WICED device boots up it will publish current state of LIGHT (LED) that will be OFF on topic
 * $shadow/beta/state/DEMO. same WICED device subscribe to topic $shadow/beta/sync/DEMO to get the latest
 * state and sync with AWS.
 *
 * NOTE : DEMO is the thing name here created in AWS. You can create new thing and change here.
 *
 * After above steps from AWS CLI you can publish and change the state of LIGHT. You can refer below command as \
 * example.
 *
 * aws iot-data --endpoint-url https://data.iot.us-east-1.amazonaws.comupdate-thing-state update-thing-state --thing-name lightbulb
 * --payload "{ \"state\": {\"desired\": { \"status\": \"ON\" } } }" output.txt && output.txt
 *
 * After successful execution of above commands you can check that LED will be turned on in WICED device and publishes
 * same with reported state.
 *
 * You can also change status of LED manually with push button on WICED and WICED board publishes its new state with
 * $shadow/beta/state/DEMO. but AWS thing shadow gives back delta because reported state is overwritten with desired
 * state and it sends delta back. This is something needs to be debugged in AWS shadow part from Amazon.
 **/

#include "wiced.h"
#include "mqtt_api.h"
#include "resources.h"
#include "gpio_button.h"
#include "cJSON.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( command, ok_message, error_message )   \
        {                                                                                   \
            ret = (command);                                                                \
            mqtt_print_status( ret, (const char *)ok_message, (const char *)error_message );\
            if ( ret != WICED_SUCCESS ){}                                               \
        }
#define THING_STATE_TOPIC                   "$aws/things/lightbulb/shadow/update"
#define THING_UPDATED_TOPIC                 "$aws/things/lightbulb/shadow/update/delta"
#define SHADOW_PUBLISH_MESSAGE_STR_OFF      "{ \"state\": {\"reported\": { \"status\": \"OFF\" } } }"
#define SHADOW_PUBLISH_MESSAGE_STR_ON       "{ \"state\": {\"reported\": { \"status\": \"ON\" } } }"
#define SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED     "{ \"state\": {\"desired\": { \"status\": \"OFF\" } ,\"reported\": { \"status\": \"OFF\" } } }"
#define SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED      "{ \"state\": {\"desired\": { \"status\": \"ON\" } ,\"reported\": { \"status\": \"ON\" } } }"

#define CLIENT_ID                           "wiced_shadow_aws"
#define MQTT_REQUEST_TIMEOUT                (5000)
#define MQTT_DELAY_IN_MILLISECONDS          (1000)
#define MQTT_MAX_RESOURCE_SIZE              (0x7fffffff)

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_ip_address_t broker_address;
static wiced_mqtt_event_type_t expected_event;
static wiced_semaphore_t semaphore;
static wiced_semaphore_t wake_semaphore;
static char* led_status = "OFF";
static wiced_mqtt_security_t security;

extern wiced_result_t aws_configure_device(void);

/******************************************************
 *               Static Function Definitions
 ******************************************************/
/*
 * A simple result log function
 */
static void mqtt_print_status( wiced_result_t result, const char * ok_message, const char * error_message )
{
    if ( result == WICED_SUCCESS )
    {
        if ( ok_message != NULL )
        {
            WPRINT_APP_INFO(( "OK (%s)\n\n", (ok_message)));
        }
        else
        {
            WPRINT_APP_INFO(( "OK.\n\n" ));
        }
    }
    else
    {
        if ( error_message != NULL )
        {
            WPRINT_APP_INFO(( "ERROR (%s)\n\n", (error_message)));
        }
        else
        {
            WPRINT_APP_INFO(( "ERROR.\n\n" ));
        }
    }
}

/*
 * Call back function to handle connection events.
 */
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    cJSON *json, *original;
    char* out;
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

            original = json = cJSON_Parse( strstr( (char*) msg.data, "{\"version\"" ) );
            if ( json != NULL )
            {
                json = cJSON_GetArrayItem( json, 1 );
                json = cJSON_GetArrayItem( json, 0 );
                out = cJSON_Print( json );
                if(out != NULL)
                {
                    if(out[0] == '"')
                    {
                        out++;
                    }
                    if(out[strlen(out)-1] == '"')
                    {
                        out[strlen(out)-1] = '\0';
                    }
                }
                cJSON_Delete( original );
                printf( "Requested LED State[%s] Current LED State [%s]\n", out, led_status );
                if ( out == NULL || strcasecmp( out, led_status ) == 0 )
                {
                    break;
                }
                else
                {
                    wiced_rtos_set_semaphore( &wake_semaphore );
                }
            }
        }
            break;

        default:
            break;
    }
    return WICED_SUCCESS;
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
    conninfo.keep_alive = 10;
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

/*
 * Unsubscribe from WICED_TOPIC and wait for 10 seconds to receive an ACM.
 */
static wiced_result_t mqtt_app_unsubscribe( wiced_mqtt_object_t mqtt_obj, char *topic )
{
    wiced_mqtt_msgid_t pktid;
    pktid = wiced_mqtt_unsubscribe( mqtt_obj, topic );

    if ( pktid == 0 )
    {
        return WICED_ERROR;
    }
    if ( mqtt_wait_for( WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED, MQTT_REQUEST_TIMEOUT * 2 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
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
 * Handles key events
 */
static void setpoint_control_keypad_handler( void *arg )
{
    WPRINT_APP_INFO(("Button is pressed"));
    wiced_rtos_set_semaphore( &wake_semaphore );
}

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    static wiced_mqtt_object_t mqtt_object;
    wiced_result_t ret = WICED_SUCCESS;
    uint32_t size_out;
    platform_dct_security_t* dct_security = NULL;

    wiced_init( );

    WPRINT_APP_INFO((" Please wait, connecting to network...\n"));
    WPRINT_APP_INFO(("(To return to SSID console screen, hold USER switch for 5 seconds during RESET to clear DCT configuration)\n"));
    wiced_rtos_delay_milliseconds( 1000 );

    for ( int i = 0; i < 25; i++ )
    {
        wiced_gpio_output_high( WICED_LED2 );
        wiced_rtos_delay_milliseconds( 100 );
        wiced_gpio_output_low( WICED_LED2 );
        wiced_rtos_delay_milliseconds( 100 );

        if ( !wiced_gpio_input_get( WICED_BUTTON1 ) )
        {
            wiced_rtos_delay_milliseconds( 5000 );

            if ( !wiced_gpio_input_get( WICED_BUTTON1 ) )
            {
                wiced_gpio_output_high( WICED_LED1 );
                WPRINT_APP_INFO(( " Dct clearing start\n" ));

                /* Write received credentials into DCT */
                struct
                {
                    wiced_bool_t device_configured;
                    wiced_config_ap_entry_t ap_entry;
                } temp_config;
                memset( &temp_config, 0, sizeof( temp_config ) );
                wiced_dct_write( &temp_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof( temp_config ) );
                wiced_rtos_delay_milliseconds( 1000 );
                wiced_gpio_output_low( WICED_LED1 );
                WPRINT_APP_INFO(( " Dct clearing end\n" ));

                break;
            }
        }
    }

    /* Configure the device */
    ret = aws_configure_device();
    if ( ret != WICED_ALREADY_INITIALIZED )
    {
        WPRINT_APP_INFO(("Restarting the device...\n"));
        wiced_framework_reboot();
        return;
    }

#ifdef USE_AWS_MQTT_CERTIFICATES
    resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_rootCA_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.ca_cert );
    //resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_cert_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.cert );
    //resource_get_readonly_buffer( &resources_apps_DIR_secure_mqtt_DIR_privkey_pem, 0, MQTT_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.key );
#endif

    /* Lock the DCT to allow us to access the certificate and key */
    WPRINT_APP_INFO(( "Reading the certificate and private key from DCT...\n" ));
    ret = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
        return;
    }
    security.cert = dct_security->certificate;
    security.key = dct_security->private_key;

    /* Memory allocated for mqtt object*/
    mqtt_object = (wiced_mqtt_object_t) malloc( WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if ( mqtt_object == NULL )
    {
        WPRINT_APP_ERROR(("Dont have memory to allocate for mqtt object...\n"));
        return;
    }
    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    wiced_hostname_lookup( "data.iot.us-east-1.amazonaws.com", &broker_address, 10000 );

    WPRINT_APP_INFO(("[MQTT] Connecting to broker %u.%u.%u.%u ...\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));

    wiced_mqtt_init( mqtt_object );
    wiced_rtos_init_semaphore( &semaphore );
    wiced_rtos_init_semaphore( &wake_semaphore );

    wiced_gpio_input_irq_enable( WICED_BUTTON1, IRQ_TRIGGER_RISING_EDGE, setpoint_control_keypad_handler, NULL );

    WPRINT_APP_INFO(("[MQTT] Opening connection..."));
    RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_conn_open( mqtt_object,&broker_address, WICED_STA_INTERFACE, mqtt_connection_event_cb, &security ), NULL, "Did you configure you broker IP address?\n" );

    /* Finished accessing the certificates */
    ret = wiced_dct_read_unlock( dct_security, WICED_FALSE );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", ret ));
        return;
    }

    WPRINT_APP_INFO(("[MQTT] Publishing..."));
    RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*)THING_STATE_TOPIC, (uint8_t*)SHADOW_PUBLISH_MESSAGE_STR_OFF ,sizeof(SHADOW_PUBLISH_MESSAGE_STR_OFF) ), NULL, NULL );

    wiced_rtos_delay_milliseconds( MQTT_DELAY_IN_MILLISECONDS * 2 );

    WPRINT_APP_INFO(("[MQTT] Subscribing..."));
    RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_app_subscribe( mqtt_object, THING_UPDATED_TOPIC , WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE ), NULL, NULL );

    while ( 1 )
    {
        /* Wait forever on wake semaphore until the wake button is pressed */
        wiced_rtos_get_semaphore( &wake_semaphore, WICED_NEVER_TIMEOUT );

        /* Toggle the LED */
        if ( strcasecmp( led_status, "ON" ) == 0 )
        {
            wiced_gpio_output_low( WICED_LED1 );
            led_status = "OFF";
            WPRINT_APP_INFO(("[MQTT] Publishing..."));
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*)THING_STATE_TOPIC, (uint8_t*)SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED ,sizeof(SHADOW_PUBLISH_MESSAGE_STR_OFF_DESIRED_AND_REPORTED) ), NULL, NULL );
        }
        else
        {
            wiced_gpio_output_high( WICED_LED1 );
            led_status = "ON";
            WPRINT_APP_INFO(("[MQTT] Publishing..."));
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_app_publish( mqtt_object, WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE, (uint8_t*)THING_STATE_TOPIC, (uint8_t*)SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED ,sizeof(SHADOW_PUBLISH_MESSAGE_STR_ON_DESIRED_AND_REPORTED) ), NULL, NULL );
        }

    }

    WPRINT_APP_INFO(("[MQTT] Unsubscribing..."));
    RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_app_unsubscribe( mqtt_object, THING_STATE_TOPIC ), NULL, NULL );

    WPRINT_APP_INFO(("[MQTT] Closing connection..."));
    RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( mqtt_conn_close( mqtt_object ), NULL, NULL );

    wiced_rtos_deinit_semaphore( &semaphore );
    WPRINT_APP_INFO(("[MQTT] Deinit connection..."));
    ret = wiced_mqtt_deinit( mqtt_object );
    mqtt_print_status( ret, NULL, NULL );
    free( mqtt_object );
    mqtt_object = NULL;

    return;
}
