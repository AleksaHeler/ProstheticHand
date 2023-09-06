# Firmware/software for the prosthetic hand project

Here you can find a folder **ProstheticHand** which is acutally [PlatformIO](https://platformio.org/) project in [Visual Studio Code](https://code.visualstudio.com/download). Flashing and debugging of the code is done using [J-Link debugger from SEGGER](https://www.segger.com/products/debug-probes/j-link/). Below is the step by step process on how to set the project up and compile it first time:
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
