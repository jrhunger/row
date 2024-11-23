# row
Replacement brainz for my (NordicTrack RW600) rowing machine.

## Folder contents
* components/hd44780 - ripped from https://github.com/UncleRus/esp-idf-lib to control the LCD
* components/esp_idf_lib_helpers - required for hd44780 and taken from the same place

## Current State
Displays the following:
* top row:
  * current resting time mm:ss (increments when wheel is stopped, resets to zero when it starts)
  * stroke rate (pulls/minute)
  * total pull count
* bottom row:
  * total active time mm:ss (increments when wheel is spinning, does not reset)
  * pwr:  ### of reed switch impulses in most recent sample interval
    * == wheel rotations * (unknown atm) number of magnets
  * distance: total # of reed switch impulses (basically dimensionless) 

## TODO
* bug: sometimes the stroke rate goes way too high (100+)
  * pull transition must be prematurely detected 
  * detection is based on derivative calculation of last 5 samples - samples are 200ms
    * calculating for midpoint (lag = 400ms)
  * ? use larger window size for derivative?
    * would increase the detection lag, unless sample is more frequent
  * ? set max stroke rate (min stroke time) and ignore detection if not reached ?
    * have seen 36 on dark horse rowing - 40 might be a good threshold
* visual guide for desired stroke rate
  * use resting time digits to make a spinning loop?
  * will need some way to set the desired rate
* control the magnetic resistance
  * standard DC motor forward/reverse with potentiometer to indicate current position

## Reference / similar projects
* [openrowingmonitor](https://github.com/laberning/openrowingmonitor) 
  * Mature and complex project in nodejs for Raspberry Pi. 
  * Trying to be physics-accurate, which is overkill for what i'm doing, but still a good reference
* [ArduRower](https://github.com/zpukr/ArduRower), which references...
* [waterrino](https://github.com/adruino-io/waterrino), which references...
* [yunrower](https://bitbucket.org/giobianchi/yun_rower/src/master/)