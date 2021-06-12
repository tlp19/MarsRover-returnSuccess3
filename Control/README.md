# Control sub-system

## Installation Guide

#### Development Environment

This code is meant to be compiled to uploaded to the ESP32 using the Arduino IDE 1.8.15

#### Adding support for the board to the Arduino Guide

In Arduino IDE, go to _File>Preferences_ and in the _Additional Board Manager URLs_ field, add "https://dl.espressif.com/dl/package_esp32_index.json". Click on OK.

Then open the board manager in _Tools>Boards>Boards Manager..._. Search for "ESP32" and install "esp32 by Espressif Systems".

Then in _Tools>Boards>_ select "DOIT ESP32 DEVKIT V1". Select the right COM port that your board is connected to (if you don't see it, you need to install the drivers at "https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers")

#### Installing the libraries necessary for compiling the code

To compile the code, you need to install an MQTT client library called PubSubClient. To do so, go in _Tools>Manage Libraries..._ and search for "PubSubClient". Install the one called "PubSubClient by Nick O'Leary".

#### Using the Serial Monitor

Open _Tools>Serial Monitor_ and make sure that the baud rate is set at 115200.
