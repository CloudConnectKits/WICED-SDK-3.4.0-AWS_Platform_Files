#
# $ Copyright Broadcom Corporation $
#

NAME := App_shadow

$(NAME)_SOURCES := shadow.c

$(NAME)_SOURCES += ../config/aws_config.c

$(NAME)_COMPONENTS := protocols/MQTT \
                      inputs/gpio_button \
                      libraries/utilities/cJSON

GLOBAL_DEFINES += USE_AWS_MQTT_CERTIFICATES

$(NAME)_RESOURCES  := apps/secure_mqtt/rootCA.pem

$(NAME)_RESOURCES += images/brcmlogo.png \
                     images/brcmlogo_line.png \
                     images/favicon.ico \
                     images/scan_icon.png \
                     images/wps_icon.png \
                     images/64_0bars.png \
                     images/64_1bars.png \
                     images/64_2bars.png \
                     images/64_3bars.png \
                     images/64_4bars.png \
                     images/64_5bars.png \
                     images/tick.png \
                     images/cross.png \
                     images/lock.png \
                     images/progress.gif \
                     scripts/general_ajax_script.js \
                     scripts/wpad.dat \
                     apps/secure_mqtt/aws_config.html \
                     config/scan_page_outer.html \
                     styles/buttons.css \
                     styles/border_radius.htc

$(NAME)_COMPONENTS += daemons/HTTP_server \
                      daemons/DNS_redirect \
                      protocols/DNS
