#
# $ Copyright Broadcom Corporation $
#

NAME := App_aws_demo_publisher

$(NAME)_SOURCES := publisher.c

$(NAME)_COMPONENTS := protocols/MQTT \
                      drivers/sensors/NCP18XH103J03RB \
					  protocols/HTTP \
					  utilities/cJSON

GLOBAL_DEFINES += USE_AWS_MQTT_CERTIFICATES

$(NAME)_RESOURCES  := apps/secure_mqtt/rootCA.pem \
					  apps/secure_mqtt/cert.pem \
					  apps/secure_mqtt/privkey.pem