

# Prerequisites

## Espressif ESP32-DevKitC 
First of all, you need ESP32-DevKitC to use this project.

## Amazon FreeRTOS environment
Before you build this project, you need to setup Amazon FreeRTOS environment according to the following document.

[Getting Started with the Espressif ESP32-DevKitC and the ESP-WROVER-KIT](
https://docs.aws.amazon.com/freertos/latest/userguide/getting_started_espressif.html)

# Download this project

```
$ git clone https://github.com/waura/iot_env_sensor.git
$ tree .
.
├── amazon-freertos
└── iot_env_sensor
```

You need to clone this repository and place beside Amazon FreeRTOS directory.

# Setup Serial Communication Port:

```
$ cd iot_env_sensor
$ make menuconfig
```

1. In that menu, navigate to **Serial flasher config** > **Default serial port**, then press enter to configure the serial port.
2. Change default port from /dev/ttyusb0 to the device file you are using for connecting to your ESP32-DevKitC.
3. Save the modification and exit.

# Build

```
$ make 
```

Binary files would be craeted under 'build/' directory if you success the build.

# Flash the image ESP32-DevKitC 

```
$ make flash
```

The image you built would be flashed to your ESP32-DevKitC and boot.

# Make Commands:

- Build and display the *Espressif IDF Configuration* only:   
  `make menuconfig`
    
- Build only:  
  `make`  
    
- Build and flash the project:  
  `make flash`  
    
- Build, flash, and display the output in the terminal:  
  `make flash monitor`  
