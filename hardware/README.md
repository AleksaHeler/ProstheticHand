# PCB & HW designs of the prosthetic hand project

For designing those we use [KiCAD](https://www.kicad.org/download/). Good tutorial can be found [here](https://www.youtube.com/playlist?list=PL3bNyZYHcRSUhUXUt51W6nKvxx2ORvUQB). Although it's for a bit older version of the KiCAD it is still usefull.

For now only two project are here:
 - **DevelopmentBoard** - First example board to test the KiCAD software and to try our hand at making our own PCB at home (TODO: update this document with findings once home-made pcb is done)
 - **ProstheticHand v1** - The actual PCB design that will be used in the prosthetic hand (for the first prototype), hopefully to be ordered from PCB manufacturer

Folders:
 - **images** - contains images of the PCBs and hardware
 - **libraries** - contains libraries used in KiCAD

Versioning:
 - v1.x -> first prototype, with v1.0 being the first design, and then iterating trough v1.1, v1.2...
 - v2.x -> todo

## Development Board

Here is the schematic of the development board:
![development_board_schematic](images/development_board_schematic.png)

It contains 4 buttons (with pullup resistors), 3 potentiometers (two trim pots, and one guitar pot), two servo connectors, JTAG connector and 12V barrel jack.

Here are some PCB screenshots:
![development_board_pcb](images/development_board_pcb.png)
![development_board_pcb_3d](images/development_board_pcb_3d.png)

And some photos of the protoboard build:
![development_board_protoboard_front](images/development_board_protoboard_front.jpeg)
![development_board_protoboard_back](images/development_board_protoboard_back.jpeg)


## Prosthetic Hand v1

Components ideas for this iteration of the board:
 - debug LED
 - 2x pot
 - oled display
 - 2x buttons
 - 4x servos
 - current shunts for servo current measurement (we have 1 ohm power resistors at hand)
 - battery voltage sensing (connected to a battery -> 4 cell Li-ion)
 - [LDO voltage regulator](https://en.wikipedia.org/wiki/Low-dropout_regulator)
 - JTAG for programming and debugging
 - external power connector (selectable between this power input or the batteries by a switch)

Used ESP32 GPIO pins:
 - Servo output -> GPIO04, GPIO05, GPIO26, GPIO27 
 - Servo current sensing -> GPIO34, GPIO35, GPIO36, GPIO39
 - Debug LED -> GPIO23
 - JTAG -> GPIO12, GPIO13, GPIO14, GPIO15
 - UART -> GPIO01, GPIO03 (using built in serial to usb chip, no our connnected hardware)
 - Buttons -> GPIO18, GPIO19
 - Pots -> GPIO32, GPIO33
 - Battery voltage -> GPIO25
 - OLED display SPI com -> GPIO21, GPIO22

This leaves us with pretty much no spare pins. So if something comes to mind, we'll have to use multiplexers for input and outputs. Or just use the method described below.

Maybe: selectable pin functions using DIP swithces or jumper header connectors (we had some 4x DIP switches at hand so we're using them): each pin can be routed to some functionality by enabling these DIP switches. For example we can use a pin as a button input or as a pot input based on the DIP swith configuration. This is purely hardware oriented configuration and we have to then configure the code to support these changes.

