#### Note! For hardware and software documentation plus in-depth Getting Started Guide, please goto : 
##### http://cloudconnectkits.org/product/avnet-bcm4343w-iot-starter-kit

# AWS IOT Demo applications
There are 4 demo applications located in the apps folder:

####  1) pub_sub:
      One for publishing when we press the button on EVB and other subscribes to same topic.
      based on message recieved subscriber toggles LED.
      
#### 2)  shadow:
      Application which sync and report device states to AWS Things Shadow.
      
#### 3) temperature_sensor_controlled_window:(BCM943364WCD1 platform)
      One executable for publishing and the other for subscribing.
      Publisher senses the room temperature and get's outside temperature from weather.com.
      Controls remote LED based on delta by publishing to topic WICED_BULB in AWS IOT Service.
      Subscriber receives the message sent for topic WICED_BULB and controls the LED.
      
#### 4) light_sensor_controlled_window:(AWS-EVB-4343W platform)
      One executable for publishing and the other for subscribing.
      Publisher senses light and controls remote LED based on the light intensity by publishing to topic WICED_BULB in AWS          IOT.Subscriber receives the message sent for topic WICED_BULB and controls the LED.

### ReadMe files 
How to guides on re-use of example code in your own application:

    BCM4343W/resources/README.txt
    BCM4343W/platforms/BCM94343W_AVN/README.txt

### Support Pages

[Broadcom Community][df1]

[Avnet CloudConnectKits][df2]

[WICED SDK 3.4.0-AWS][df3]


   [df1]:<https://community.broadcom.com/welcome>
   [df2]:<http://cloudconnectkits.org/>
   [df3]:<https://community.broadcom.com/docs/DOC-2484>