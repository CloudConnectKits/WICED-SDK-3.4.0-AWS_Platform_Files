# AWS IOT Demo applications. 
There are 4 demo applications that can be found in the apps directory:

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

### Readmes, how to use them in your own application can be found here:

    BCM4343W/resources/README.txt
    BCM4343W/platforms/BCM94343W_AVN/README.txt

### Visit here for support
[Broadcom Community][df1]

[Avnet Cloudconnectkit][df2]
    
[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does it's job. There is no need to format nicely because it shouldn't be seen. Thanks SO -
  
   [df1]:<https://community.broadcom.com/welcome>
   [df2]:<http://cloudconnectkits.org/>