Firmware for GXB172 Flashlight Driver

Major changes in compare with [original](https://github.com/loneoceans/gxb172-attiny841 "loneoceans/gxb172-attiny841"):
1. EPPROM not used.
  You can switch the mods in loop, like it was in original firmware, but each time starting from default mode.
2. No candle mode or some other configuration, or jumpers.
  Only five brightness  modes: 1, 50mA, 250mA, 1000mA, 5000mA
3. External temperature sensor is not required, it is already in MCU.
  But it will be preferable if it is will be installed.
4. All (temperature and battery voltage) external data filtered with median filter.
5. Decreased low battery cap to 2.5 V. 
6. Two PID controls for the themperature and battery voltage.
7. Smoother brightness and faster PID response.
8. PID is working in the all brightness modes.
