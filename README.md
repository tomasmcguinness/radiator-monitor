# Radiator Monitor

Not much to see right now as I've just published this!

To compile it, you'll need to use ESP-IDF for the Bridge project and Nordic nRF Connect for the sensor

It's designed for use on an ESP32-C3 and Nordic nRF52840. I've been using the Seeed XIAO boards.

Both of these are available as add-ons to VSCode and that's how I've been building, compiling and flashing.

You'll need a J-Link piece of hardware to flash the Nordic boards at this time.

For an overview, please watch my YouTube video - https://youtu.be/YUV8U6BC5oQ

## Overview

There are two projects in this solution: Bridge and Sensor

### Bridge

The bridge is responsible for connecting to WiFi and for listening for the various updates that come in via the BLE Mesh. It hosts a web application, which can be access via radmon.local once it has been connected to WiFi.

### Sensor

The sensor is physically connected to your radiators and will transmit the current temperature readings every 5 seconds.
