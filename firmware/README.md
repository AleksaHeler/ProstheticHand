# Firmware/software for the Prosthetic Hand project

Within this directory, you can find the **ProstheticHand** [PlatformIO](https://platformio.org/) project created in the [Visual Studio Code](https://code.visualstudio.com/download) editor.  
The [SEGGER J-Link debug probe](https://www.segger.com/products/debug-probes/j-link/) is used for code flashing and debugging.  
Below is the step by step process on how to set the project up and compile it for the first time:
 - __Install [Visual Studio Code](https://code.visualstudio.com/download)__
 - __Install the [PlatformIO](https://platformio.org/install/ide?install=vscode) plugin__ for Visual Studio Code
 - __Install the [J-Link Software](https://www.segger.com/downloads/jlink/)__ from SEGGER (Choose the installer based on your operating system, leave all settings at their default values during installation)
 - __Clone the project's git repository__
 - __Open the _ProstheticHand_ project__ from PlatformIO's home screen
 - When opening the project for the first time, PlatformIO should automatically update and download necessary dependencies and configure the project.
 - Try to __build the code__. It should work without issues.
 - If there are issues, __ensure that the correct USB driver is installed__, as it is required for the J-Link debugger to function.
  For Windows, we used the WinUSB driver. If it's missing, you can use [Zadig](https://zadig.akeo.ie/) to install it. (Our example driver version is v6.1.7600.16385.)
 - With the J-Link connected and the ESP32 powered on, you should be able to upload the code using the J-Link debugger, as well as debug it.

If you want to use the serial monitor with the ESP32, you may need to install drivers for its onboard USB to UART bridge:
 - __Identify the USB to UART bridge__ on your ESP32 board
 - __Find and install the driver__
 You can find most USB to UART bridge chips included on ESP32 development boards listed [here](http://esp32.net/usb-uart/), together with the necessary drivers
 
 Note: You can install the [Serial Monitor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) extension for Visual Studio Code, which provides a more organised interface compared to PlatformIO's built-in serial monitor.


## Code guide / developer guide

All the source code files are in **src** folder. In there you can find a couple of folders/files:
 - main.cpp - entry point of the application
 - config - contains header files for defines and typedefs, version information and includes used in the whole project 
 - include - contains 'libraries' used in project. In quotations since they are not exatcly libraries but can be considered as such let's say
 - drivers - contains the actual components of the program, functional codes that behave as components/objects (each has init and handle function and implements its own logic)

All .cpp files (for example **main.cpp**) shall have a **main_e.h** (external) and **main_i.h** (internal). Using this approach we can have private and public global variables and functions, which can be useful in such a project. For example we can have 'init' and 'handle' functions declared in external header file and helper functions used for internal logic defined in internal header. Next, the main module (or anywhere this code is used) can include only the _e.h file, and thus only have access to what is needed to use the module, and not the internal logic stuff.

**Naming convention** used: *main_f_Handle_v*, *btn_BtnStates_u8*, *pot_g_PotConfig_s*
 - first is the module name (main, btn, pot, srv...)
 - then kind of literal (f - function, g - global var, s - struct, c - constant...)
 - then the name (in upper camel case - UpperCamelCase)
 - finally the type, or return type for function (v - void, p - pointer, u32 - unsigned 32bit int, f64 - 64bit float...)

We have main.cpp as entry point, and then it manages all the other drivers (logical components: buttons driver, potentiometer driver, servo driver...). This means that in the main program, we have two functions: **main_f_Init_v** and **main_f_Handle_v**. In the init function, the main calls each drivers init function (*btn_Init_v*, *pot_f_Init_v*) once during boot, and then goes on to call the handle function in loop while the device is running (*btn_Handle_v*, *pot_f_Handle_v*). Each module handles internal logic on its own, and some modules, for example servo driver, take other modules outputs (like button and pot states) and use them, like in the servo driver it uses pot to set the angle of the servo. This is used in order to be more modular and to have easier time implementing smaller software components instead of spaghetti code.

**Main code** has init and handle functions, and is primarily focused on calling each modules init and handle function. But it does that in a way that we have a **10ms repeating loop**, where during each 1ms subdivision we handle some of the functionality. So for example during first 1ms it will call one drivers handle function, then in the next 1ms it will call another and so on, until we loop back after 10ms total. This is done in order to better distribute load over time, and to provide us with easier way of measuring runtime durations and to find parts of code that take too much time, and in that case, perhaps distribute it over couple of 1ms tasks.


## Using the program

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
Here we can see the above-mentioned 10 x 1ms tasks with each tasks current execution time, min and max times. Also each driver can implement its own serial debug function where it displays useful data to serial (button states, pot values, if servo driver is running...)
Hopefully we will soon update this to use json format, as it will then be easier to use with some other program on the PC for visualising the data.
