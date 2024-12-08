# row
Replacement brainz for my (NordicTrack RW600) rowing machine.

## Folder contents
* components/hd44780 - ripped from https://github.com/UncleRus/esp-idf-lib to control the LCD
* components/esp_idf_lib_helpers - required for hd44780 and taken from the same place

## Current State
Displays the following:
* top row:
  * state-dependent:
    * resting: current resting time mm:ss (increments when wheel is stopped, resets to zero when it starts)
    * active: pacer bar indicating target stroke rate
  * stroke rate (pulls/minute)
  * total pull count
* bottom row:
  * total active time mm:ss (increments when wheel is spinning, does not reset)
  * power:  smoothed and arbitrarily scaled ### of reed switch impulses in most recent sample intervals
  * distance: total # of reed switch impulses (basically dimensionless) 

## TODO
* control the target stroke rate
  * quadrature encoder?
* control the magnetic resistance
  * standard DC motor forward/reverse with potentiometer to indicate current position

## Reference / similar projects
* [Sovitsky-Golay filters](https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter)
  * used to get derivative of impulse delay times for stroke detection
  * also smoothing the "power" samples, though a standard average would be fine here
* [openrowingmonitor](https://github.com/laberning/openrowingmonitor) 
  * Mature and complex project in nodejs for Raspberry Pi. 
  * Trying to be physics-accurate, which is overkill for what i'm doing, but still a good reference
* [ArduRower](https://github.com/zpukr/ArduRower), which references...
* [waterrino](https://github.com/adruino-io/waterrino), which references...
* [yunrower](https://bitbucket.org/giobianchi/yun_rower/src/master/)
