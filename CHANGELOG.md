# Changelog

## [4.0] - 2018-10-17
### Added
 - Kicad PCB design gerber/sources.
 - PIC18F24Q10 support.

### Changed
 - Makefile now use PICKIT4 and IPECMD
 - Schematic now in KiCad format.

## [3.5] - 2018-10-06
### Added
 - Pre-defined button layout.
 - Two buttons special mapping options for A+C-Down and B+C-Left.
 - Turbo buttons special feature.

### Changed
 - Serial bridge ported to PIC18F14K22.

## [3.4] - 2018-06-21
### Added
 - Joystick response curve per axis and layout.
 - Menu LED indicator.

### Changed
 - Answer N64 poll within WaveBird init.
 - Mute joysticks input while in menu.

## [3.3] - 2018-06-05
### Added
 - Interrupt based reply.
 - Support 1 player 2 controllers control style.

## [3.2] - 2018-06-01
### Changed
 - N64 controller pin on PORTB.
 - Joystick scaling menu option.

## [3.1] - 2018-02-05
### Added
 - Slot bypass mode. (Fixes #10)
 - GameCube joysticks scaling. (Fixes #7)

### Changed
 - Ported Python2 scripts to Python3.

## [3.0] - 2018-01-03
### Added
 - Support PIC18F14K22 microcontroller.
 - Real-time CRC calculation. (Fixed issue #1)
 - Axes remapping.
 - Layout modifier function.

### Changed
 - Configuration menu.
 - EEPROM flexibility.

### Removed
 - Support PIC12F683 microcontroller.

## [2.0] - 2011-04-20
### Added
 - Support PIC12F683 microcontroller.
 - Support identify command 0xFF. (Fixed issue #2)
 - Enable/Disable Rumple Pak emulation. (Enhancement #4)
 - Support WaveBird. (Enhancement #5)
 - Support 4 mapping layout.

### Removed
 - Support for PIC16F84A & PIC12F629.

## [1.0] - 2004-08-07
 - Initial version.

