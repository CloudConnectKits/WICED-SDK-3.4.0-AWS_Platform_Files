#
# Copyright 2015, Broadcom Corporation
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

NAME := App_aws_demo_subcriber

$(NAME)_SOURCES := subscriber.c \
                   ../../shadow/aws_config.c \
                   ../../shadow/aws_common.c

$(NAME)_INCLUDES   := ../../shadow

$(NAME)_COMPONENTS := protocols/MQTT \
                      libraries/utilities/cJSON

WIFI_CONFIG_DCT_H := wifi_config_dct.h
APPLICATION_DCT := ../../shadow/aws_config_dct.c

$(NAME)_RESOURCES  := apps/aws_iot/rootca.cer

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
                     apps/aws_iot/aws_config.html \
                     config/scan_page_outer.html \
                     styles/buttons.css \
                     styles/border_radius.htc

$(NAME)_COMPONENTS += daemons/HTTP_server \
                      daemons/DNS_redirect \
                      protocols/DNS
