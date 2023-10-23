# PCB & HW designs of the prosthetic hand project

For designing those we use [KiCAD](https://www.kicad.org/download/). Good tutorial can be found [here](https://www.youtube.com/playlist?list=PL3bNyZYHcRSUhUXUt51W6nKvxx2ORvUQB). Although it's for a bit older version of the KiCAD it is still usefull.

## Prosthetic Hand v2

We're using EPS32-S3:
 - https://api.riot-os.org/group__cpu__esp32__esp32s3.html

Will have:
- still use full-size ESP32 S3 Devkit C boards
- 2 debug LEDs (one main RED LED, one green LED for sensor actuating)
- 5V dropdown regulator
- battery voltage sensing
- 4 dip switches for configuration
- 2 sensor connections (will be on another board with 2x 9V batteries)

Ideas:
- 3 potentiometers (one big, two trim pots)
- 2 buttons

- 3 servo motors, current sensing for each and MOSFET to control the power/current they get
- MOSFETs controlled by op-amps, where uC gives them using PWM signal and sets the max current


KiCad 7 ESP32 PCB Design Full Tutorial: https://www.youtube.com/watch?v=b-7bMl6fJio
RF + MCU PCB Design Review: https://www.youtube.com/watch?v=71bW_sKZIZw
