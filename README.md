# roboscle
Open-source FOC driver and DAQ system for robotic projects. 

# Dependencies
## Motor Driver
The motor driver is based on [B-G431B-ESC1 Discovery kit](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) and the firmware is developed using STM MCSDK libraries. The following versions of the library and toolchains are used in this project:
- **STM32CubeIDE V1.12.0**
- **STM32CubeMX V6.8.0**
- **STM32CubeG4 Firmware Package V1.5.0** 
- **STM Motor Control WorkBench V6.1.1**

## CAN-UDP DAQ
The DAQ system used to communicate with the driver is based on the [Teensy 4.1 with Ethernet kit](https://www.amazon.com/PJRC-Cortex-M7-iMXRT1062-Microcontroller-Development/dp/B08RSCFBNF) and the firmware is developed using the Arduino V2.0.4 and Teensy board support package V1.58.0.

# Directory Structure
- CubeIDE project: 
`firmwares\motor_driver\solo_foc\STM32CubeIDE\.project`
- STM MCSDK Projet: `firmwares\motor_driver\solo_foc.stwb6`
- STM CubeMX Project: `firmwares\motor_driver\solo_foc\solo_foc.ioc`
