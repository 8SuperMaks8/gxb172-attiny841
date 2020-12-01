Firmware for GXB172 Flashlight Driver

Major changes in compare with [original](https://github.com/loneoceans/gxb172-attiny841 "loneoceans/gxb172-attiny841"):
* EPPROM not used.
  Mod can be switched in loop, like it was in original firmware, but each time starting from default mode.
* No candle mode or some other configuration, or jumpers.
  Only five brightness  modes: 1, 50mA, 250mA, 1000mA, 5000mA
* External temperature sensor is not required, it is already in MCU.
  But it will be preferable if it is will be installed.
* All (temperature and battery voltage) external data filtered with median filter.
* Decreased low battery cap to 2.5 V. 
* Two PID controls for the themperature and battery voltage.
* Smoother brightness and faster PID response.
* PID is working in the all brightness modes.
