README

Envirobot 2.0 Module STM32 Software

This repository saves the STM32 software in modules for the Envirobot 2.0 project.
Implemented with STM32CubeMX 6.6.1 and STM32CubeIDE 1.10.1.

Files:
/Core/main.cpp: main file run by the STM32. Please rename to main.c before regenerating configuration files with CubeMX. Then, name it back to main.cpp.

/Src/: all core classes used by the software

/Patform/: platform specific functions to make the /Src/ folder usable in other projects.

/User/: user code to be run in parallel of the core firmware by the mean of freeRTOS tasks. This is the only folder the user should have access to.

/Configuration/Configuration.h: main configuration file to activate or deactivate software capabilities at the flash procedure. Please comment the lines to remove to save flash space.

/Configuration/Configurations/: folder that saves the default configuration of core classes from /Src/. This can be used to reconfigure key parameters easily at compile-time.

/Configuration/Definitions/: save all definition files to refer to when coding with core classes.

/Configuration/Hardware/: Hardware register maps of used sensors

/Configuration/RegisterMaps/: register maps of core classes. Use these files to communicate, configure and control the platform.