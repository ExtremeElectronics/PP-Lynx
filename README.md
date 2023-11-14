# PICO LYNX EMULATION
```
____________________________________________
|                                          |
|                            _____         |
|  PLYNX              _____ |     |        |
|  (PicoLynx)        |     ||     |        |
|                    |     ||_____| _____  |
|                    |_____|       |     | |
|  Extreme Kits  _____             |     | |
|               |     |  _________ |_____| |
|               |     | |         |        |
|               |_____| |         |        |
|  Kits at              |         |        |
|  extkits.uk/PLYNX     |         |        |
|  2023                 |_________|        |
|                                          |
|__________________________________________|
```

A port (mostly) of the EPS32 PALE Emulator by Charles Peter Debenham Todd   https://github.com/retrogubbins/PaleESP32VGA to a PIPico processor and a 240x320 display


A big thanks to the details  and manuals in https://github.com/ukscone/Camputers-Lynx Russell Davis Lynx Archive

Z80 Emulation via EtchedPixels z80 emulation kit  https://github.com/EtchedPixels/EmulatorKit origonally by © Gabriel Gambetta (gabriel.gambetta@gmail.com) 2000 - 2014

To run on the PicoPuter kit Available (soon) from extkits.co.uk

Running programs in the TAP format are in the SD directory, for more info see the readme there. 

#Function Keys 
F1 - Escape
F2 - load taps file from SD
F9 - Z80 reset (only)

Current setup in a 3D printed case. 
![421b32f81baee515](https://github.com/ExtremeElectronics/PP-Lynx/assets/102665314/4a728921-2a83-44ab-8db5-c439b738eea2)

## ToDo 
Errors on file load/save (file still loads/saves, but console get an error)
Disk support (may need more memory than the Pico has) 

