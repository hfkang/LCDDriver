# LCDDriver
Actually handles all of the code. Not just LCD

### LCDDriver.asm
The star of the show. At least, the eldest child. This module controlls the pin lines leading to the LCD, and is responsible for appropriate signal timing, command/instructions, and data formatting. 
The subroutines generally aren't called directly elsewhere in the program, but rather through macros which handle the setup and configuration as needed. This was mostly done to take advantage of parameters
in macros, which facilitate the coding process.

### Interface.asm
This module is the main entry point for the program, as it contains the setup and data required for the user interface. It makes extensive interfacing with the keyboard
and LCD peripherals, and is responsible for system wide configuration (config bits, control registers, etc..). The program calls upon routines in the other modules 
to setup their respective hardware devices as necessary. 

### Sensors.asm
This module is where most of the datacollection occurs. The IR, Ultrasound and Encoders all store their values using the routines defined
in this file. The subroutines for data logging are on the shorter side, so that the PIC can read the sensors as quickly as possible. When needed the processor
will read the appropriate registers in the main control loop. 

### Motors.asm
This is where the h bridge and stepper driver are controlled. The PWM outputs are controlled here, and the stepper driver is timed here. 
Once PD and stepper obstacle avoidance methods are developed they'll go here. 

### MACROS.inc
This file contains the myriad of helper macros that simplify the development process. The use of macro parameterization helps make 
assembly resemble modern functional programming a little more, and enables greater flexibility in reusing subroutines. While the use of macros
requires more program memory, the PIC has more than enough (8K) and the space is well worth the speedup it enables. 
