# LoopingLouino Project

find pictures and further description here: https://redlabs.de/blog/looping-louino/

## Repository Overview
- **documentation**
  - pictures of this project
- **firmware**
  - Arduino Project
- **hardware**
  - **KiCAD** Project Folder including schematic, PCB layout and BOM
  - **schematic** and fabrication layer drawing as PDF
  - **datasheet** of used components
  
## Hardware
- Order PCB: https://aisler.net/p/MKIYMUSK
- Assemble PCB (find BOM in \hardware\schematic)
- Rework PCB as described in the schematic

## Firmware

### Flash Arduino bootloader to your Atmega328

1. Open Atmel Studio (now Microchip Studio)
2. Start the Tool Manager. Connec your Atmel ISP to your PC and to the PCB (CON2, ISP 6-pin).
3. Select Tool and Device. Apply & Read.
4. "Fuses": Set Fuses (according to picture "Arduino Fuses.pg"). Program fuses to Device.
5. "Memories": For Flash, select the "optiboot_atmega328.hex" file. Program&Verify.

... find pictures and data here: \firmware\Arduino Bootloader


### Upload the LoopingLouino Fimrware 

Prepare:
- Get an USB/UART converter
- Connect the UART to the PCB (Pinheader P1, or use the Audio Jack J1)

Upload:
1. Open the firmware folder in VSCode
2. Install the "PlatformIO IDE" extension
3. Open the folder "180223-094129-pro16MHzatmega328" with the extension
4. Build and Upload the project.

... find other projects: https://redlabs.de/

### Debug the Firmware

In main.cpp set "#define DEBUG 1" to get serial debug information (8n1 19200)

### Adaptions in the Firware

According to the used Supply Voltage, set: #define SPEED_CORRECTION_FACTOR (main.h)

According to the number of LEDs used, set: #define NUM_LEDS (LEDcntrl.h)

## Mechanics

//TODO: add pictures

Open all screws (short / long)

remove black cable (between motor and batteries)
remove red cable (between switch and batteries)
remove battery clips

add new black cable to motor (20cm)
add new red cable to switch (20cm)

cut through plastic (battery holder)
drill through plastic frame (for DC Jack, Button and optional RC-Jack)

cut digital LED strip and attach 3-wire ribbon cable (20LEDs, or other number)

solder 3-wire LED cable to PCB
solder 2-Wire Motor-and-Switch cable to PCB