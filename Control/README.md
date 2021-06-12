# Control sub-system

## Installation Guide

#### Development Environment

This code is meant to be compiled to uploaded to the ESP32 using the Arduino IDE 1.8.15

#### Adding support for the board to the Arduino IDE

In Arduino IDE, go to _File>Preferences_ and in the _Additional Board Manager URLs_ field, add "https://dl.espressif.com/dl/package_esp32_index.json". Click on OK.

Then open the board manager in _Tools>Boards>Boards Manager..._. Search for "ESP32" and install "esp32 by Espressif Systems".

Then in _Tools>Boards>_ select "DOIT ESP32 DEVKIT V1". Select the right COM port that your board is connected to (if you don't see it, you need to install [these drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)).

#### Installing the libraries necessary for compiling the code

To compile the code, you need to install an MQTT client library called PubSubClient. To do so, go in _Tools>Manage Libraries..._ and search for "PubSubClient". Install the one called "PubSubClient by Nick O'Leary".

#### Modifying the network credentials

To make the ESP32 connect to the internet, you need to input your WiFi SSID and password in the header file "wifiRover.h" as `const char *SSID` and `const char *password` respectively.

Note: if the IP address of the MQTT Broker ever changes, this also needs to be updated in the code in the header file "mqttRover.h" as `const char *mqttServer`.


#### Compiling, uploading, running, and using the Serial Monitor

Now to upload the code to your board, click on "Upload".
Wait for the code to compile and upload to your board. Once this is done, you can disconnect your computer from the board. The program runs automatically, but you can reset the board by pressing the __EN__ button on the board itself to start it again from the beginning.

If you want to check the internal info from the program, keep your computer plugged into the board. Open _Tools>Serial Monitor_ and make sure that the baud rate is set at 115200. You should now see messages print on the screen.
