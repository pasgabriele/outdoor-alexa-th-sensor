# outdoor-alexa-th-sensor

This device is an outdoor battery powered temperature & humidity sensor compatible with Amazon Alexa. To permit Amazon Alexa integration is used the Sinric Pro service (https://sinric.pro).

## Features

1 Battery powered
2 Compatible with Amazon Alexa
3 Configurable Deep Sleep time to save battery life

## Hardware

The hardware composition is the following:
* NodeMCU ESP32
* DHT22 Temperature & Humidity sensor
* 18650 3.7V Li-Ion battery

### Wiring

 ESP32     | DHT22
 --------- | -----
 3V3       | +
 GND       | -
 D5 (GPIO5)| OUT
 
 ## Software
 
 The sketch is 

