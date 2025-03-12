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
**Order PCB:** https://aisler.net/p/MKIYMUSK


## Firmware

### Flash Arduino bootloader to your Atmega328

1. Open Atmel Studio (now Microchip Studio)
2. Start the Tool Manager. Connec your Atmel ISP to your PC and to the PCB (CON2, ISP 6-pin).
3. Select Tool and Device. Apply & Read.
4. "Fuses": Set Fuses (according to picture "Arduino Fuses.pg"). Program fuses to Device.
5. "Memories": For Flash, select the "optiboot_atmega328.hex" file. Program&Verify.

... find pictures and data here: \firmware\Arduino Bootloader


### Upload the LoopingLouino Fimrware 

Prepare to upload:
1. Get an USB/UART converter
2. Connect the UART to the PCB (Pinheader P1, or use the Audio Jack J1)

1. Open the firmware folder in VSCode
2. Install the "PlatformIO IDE" extension
3. Open the folder "180223-094129-pro16MHzatmega328" with the extension
4. Build and Upload the project.

... find other projects: https://redlabs.de/

### Debug the Firmware

In main.cpp set "#define DEBUG 1" to get serial debug information (8n1 19200)