# ProstheticHand Application Software
 
## Installation and setup
This project is made using [PlatformIO](https://platformio.org/) with [Visual Studio Code](https://code.visualstudio.com/download).
Flashing and debugging of the code is done using [J-Link debugger from SEGGER](https://www.segger.com/products/debug-probes/j-link/).

Here is a guide on how to prepare project for developers:
 - Install [Visual Studio Code](https://code.visualstudio.com/download), and install [PlatformIO](https://platformio.org/install/ide?install=vscode) plugin for it
 - Install [J-Link Software](https://www.segger.com/downloads/jlink/) from SEGGER (Windows 64-bit Installer, leave everything at default while installing)
 - You may need to have Arduino IDE installed (to be tested)
 - Clone our projects git repository
 - Open the "ProstheticHand" project from PlatformIOs home screen
 - First time opening the project, PlatformIO should update and download something and configure it
 - Then if you try to build the code, it should work
 - J-link should have on it WinUSB (our example v6.1.7600.16385) driver installed. (if not, install it using [Zadig](https://zadig.akeo.ie/))
 - Now, if the J-link is connected and the ESP32 receives power, you should be able to upload the code using J-link as well as debug the code

If you want to have serial monitor with the ESP32, you need the drivers for its on-board USB to UARD IC:
 - Download the driver from [here](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads), under CP210x Windows Drivers and install
 
## Schematic and wiring
TODO: Create schematic for example board with buttons, potentiometer, servo, JTAG debugger...

## Code guide
TODO: Create a guide on how to code and comment everything, and the architecture of the project 

If serial degbug is enabled, the output to console for now looks like this:
```
----------------------------------------
 > runtimes (in microseconds):
    ├─[0] curr: 14, min: 13, max: 73
    ├─[1] curr: 715, min: 702, max: 843
    ├─[2] curr: 24, min: 23, max: 81
    ├─[3] curr: 1, min: 1, max: 1
    ├─[4] curr: 1, min: 1, max: 1
    ├─[5] curr: 1, min: 1, max: 1
    ├─[6] curr: 1, min: 1, max: 1
    ├─[7] curr: 1, min: 1, max: 1
    ├─[8] curr: 1, min: 1, max: 1
    ├─[9] curr: 1, min: 1, max: 1
btns:  0  0  0  0
pots:  179.96  30.22  0.00
servo working
```