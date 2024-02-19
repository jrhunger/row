# row
Replacement brainz for my (NordicTrack RW600) rowing machine.

## Folder contents
* components/hd44780 - ripped from https://github.com/UncleRus/esp-idf-lib to control the LCD
* components/esp_idf_lib_helpers - required for hd44780 and taken from the same place

## Current State
Displays the following:
* time: mm:ss of active time (only increments when wheel is spinning)
* pwr:  ### of reed switch impulses per half-second
  * == wheel rotations * (unknown atm) number of magnets
* distance: total # of reed switch impulses (basically dimensionless) 

## Plans
* calculate and display stroke rate
  * detect Stroke (impulse interval decreasing)
  * detect Recovery (impulse interval increasing)
* control the magnetic resistance
  * standard DC motor forward/reverse with potentiometer to indicate current position

## Reference / similar projects
* [openrowingmonitor](https://github.com/laberning/openrowingmonitor) 
  * Mature and complex project in nodejs for Raspberry Pi. 
  * Trying to be physics-accurate, which is overkill for what i'm doing, but still a good reference
* [ArduRower](https://github.com/zpukr/ArduRower), which references...
* [waterrino](https://github.com/adruino-io/waterrino), which references...
* [yunrower](https://bitbucket.org/giobianchi/yun_rower/src/master/)