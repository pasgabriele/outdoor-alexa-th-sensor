# outdoor-alexa-th-sensor

This sketch is used to create an outdoor battery powered temperature & humidity sensor compatible with Amazon Alexa.

## Hardware

The hardware composition is the following:
* NodeMCU ESP32
* DHT22 Temperature & Humidity sensor
* 18650 3.7V Li-Ion battery

### Wiring

 18650 | ESP32
 ----- | -----
 +     | VIN
 -     | GND

 ESP32     | DHT22
 --------- | -----
 3V3       | +
 GND       | -
 D5 (GPIO5)| OUT

