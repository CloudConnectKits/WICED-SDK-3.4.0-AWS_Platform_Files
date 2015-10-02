#
# $ Copyright Broadcom Corporation $
#

NAME := App_aws_demo_subcriber

$(NAME)_SOURCES := subscriber.c

$(NAME)_COMPONENTS := protocols/MQTT \
					  libraries/utilities/cJSON

GLOBAL_DEFINES += USE_AWS_MQTT_CERTIFICATES

$(NAME)_RESOURCES  := apps/secure_mqtt/rootCA.pem \
					  apps/secure_mqtt/cert.pem \
					  apps/secure_mqtt/privkey.pem