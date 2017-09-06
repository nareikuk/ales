# ales
Altitude Limiting Electronic Switch for model electric glider competitions

This is still under early development even though it has been under construction since 2016. The aim is to produce a cheap,
easy to construct and simple to use altitude limiting switch for electric motor launched model gliders for competition use.
The schematics, PCB and firmware are all provided. I may be able to supply blank PCBs or even fully populated PCBs in the future,
once the bugs have been ironed out and I'm happy that the project has reached a production quality level.

Note that currently the firmware is being debugged due to an elusive issue with the altitude cutoff not always working.

The current features of the ALES v2 include:
1) Pre-set altitude cut-off of 100m, 150m or 200m which are the three standards used in competition.
2) Motor run time limited to 30 seconds if pre-set altitude has not been reached before then
3) Auto-calibrates to receiver pulse timing at start up
4) Auto-calibrates altitude of launch field
5) After cut-off, if throttle stick is retuned to idle, the ALES will re-enable motor power after three seconds for those landing emergencies
6) Altitude sampling at 1kHz with better than 0.2m resolution
7) Very small and light hardware profile
8) Works with HV or standard servos and is powered from the receiver, drawing around 2mA
9) 32-bit STM32F0 microcontroller and BMP280 pressure sensor provide the core functionality
10) Single-sided load for all surface mount components

Future features once v2 has reached production quality will include anti-zoom control and field based configuration.
