# Cube64

## Introduction
This is a project to build an adaptor for using GameCube controllers
with N64 consoles. The motivation behind this is that the N64 has some
really great games, but the controller hardware is rather bad- the
joysticks wear out quickly, and the layout is odd. The GameCube
controllers, however, have a superset of the features and a much
better joystick.

A secondary goal of this project is to more thoroughly reverse engineer
the protocols involved and create reusable libraries for interfacing PIC
microcontrollers to N64 and GameCube devices. The very basics of reverse
engineering the two protocols were done before Micah (@scanlime) started this project,
but she quickly discovered that there's much more to the protocol. We believe
this project may be the first one to completely reverse engineer the
protocol used by standard N64 controllers, including peripherals like the
Controller and Rumble Paks.

All code here is released under the GPL. This means that if you have basic
electronics skills and a way to program the microcontroller, you can build
your own cube64 adaptor easily. The hardware is extremely simple, and all
the complexity of reverse engineering and writing code is done for you ;)

## Discussion

Check out the mailing list: <https://groups.google.com/forum/#!forum/cube64/>

## Usage

The completed adaptor plugs into an N64 console, and a regular GameCube
controller or WaveBird receiver plugs into the adaptor.
The default button mapping is as follows:

GameCube | N64
-------- | ---
Analog stick | Analog stick
D-pad | D-pad
A | A
B | B
Z | Z
Start | Start
R | R
L | L
C-stick | C buttons
X | C-down
Y | C-left

### Main menu

All the configurable feature of the adaptor can be accessed thought this menu.
To access the menu:

   1. Hold down the analog L and R buttons down most of the way, but not enough to
      push the end-stop buttons, and press Start.

Once in the menu the N64 will receive blank status packet regardless of buttons
pressed on the GameCube controller. An invalid selection in the menu will exit
the menu.

### Menu options

#### Axis & Button remapping

The button mapping is stored in non-volatile memory, and may be modified during
gameplay. To change the mapping for one button or axis direction:

  1. While in the main menu, press Start again.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press the button you wish to remap. This includes any button or axis direction.

  4. Press the button you want to map it to. This button is always interpreted
     according to the above defaults.

  5. If the mapping was successful and you have rumble power connected, you should feel
     a brief rumble.

#### Special function

It's possible to map a special function to a button instead of a N64 button.

  1. While in the main menu, press Y.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press the button you wish to set the function to.

  4. Press the button corresponding to the special function you want. This button is always
     interpreted according to the defaults below.

  5. If the mapping was successful and you have rumble power connected, you should feel
     a brief rumble.

##### Function available:

GameCube | Function
-------- | --------
D-pad | Temporary overwrite current mapping layout <br/>with layout corresponding to the D-pad <br/>direction until button is released.

#### Mapping analog trigger axes

By default button remapping and special function will only map to the digital trigger buttons.
Pressing X before either menu options will allow mapping to the analog trigger axes.

  1. While in the main menu, press X.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press either Start or Y. See *Axis & Button remapping* or *Special function* sections for next steps.

#### Mapping layout reset

To reset the current active mapping layout to the above defaults:

  1. While in the main menu, press Z.

  2. If you have the rumble power connected and the reset was successful, you should
     feel a brief rumble.

#### Adaptor accessory mode

The adaptor can either emulate a Rumble Pak, let all controller slot command
bypass to a real N64 controller or display the slot as empty.
To switch between the modes:

  1. While in the main menu, press D-pad UP.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press D-pad UP for bypass mode, D-pad LEFT for Rumble Pak mode or D-pad RIGHT
     for emptied slot controller.

  4. If you have the rumble power connected and the mode change was successful, you should
     feel a brief rumble.

This setting is volatile. Adaptor always starts with Rumble Pak mode.

Empty mode is useful for some game that report error with a Rumble Pak.
For those games, without even invoking the main menu you may simply hold D-pad RIGHT
on power on to disable the Rumble Pak mode and avoid the error.

Holding D-pad UP at boot will set bypass mode as well.

Setting bypass mode while no N64 controller is present in the secondary adaptor port will make
the adaptor reset.

#### Mapping layout selection

It is possible to save up to 4 sets of mapping layouts so you can easily use different keys
mapping in different games. To change between the layout does the following:

  1. While in the main menu, press D-pad LEFT.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press one of the 4 D-pad directions which correspond to the 4 layout available.

  4. If you have the rumble power connected and the layout change was successful, you should
     feel a brief rumble.

All layout by default hold the default button configuration. Just select one of 4
layouts and then change the button configuration using the button mapping key combo
described above.

When you select a layout, it is saved into the EEPROM so the next time you power up
the adaptor the last used will be reloaded automatically.

#### GameCube joysticks scaling

By default the joysticks value from the GameCube controller are scale to match the N64
joystick value range. This can be disabled to use the GameCube range instead (< v3.1 behavior).
To toggle on/off the scaling function on a global basis (affect all mapping):

  1. While in the main menu, press D-pad LEFT.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press X to toggle on/off the scaling function.

  4. If you have the rumble power connected and the layout change was successful, you should
     feel a brief rumble.

This option is saved into the EEPROM so the next time you power up the adaptor the last 
used setting will be reloaded automatically.

#### Calibration

The adaptor handles calibration in almost the exact same way a GameCube does. If
your joystick is drifting or unresponsive during gameplay, you should be able to
recalibrate it by performing any of the following actions without touching the
joystick, C-stick, L button, or R button:

  1. Hold down X, Y, and Start for a couple seconds during gameplay.

  2. Unplug the controller from the adaptor and plug it back in.

  3. Unplug the adaptor from the N64 and plug it back in.

  4. Turn off the N64 then turn it back on. (The reset button won't
     recalibrate, since the adaptor doesn't lose power or get reinitialized)

## Hardware

There are currently one supported version of the Cube64 hardware which uses
a PIC18F14K22 microcontroller, this MCU is fast enough to be able to implement
the full CRC computation. Two options are available for 5V supply:

  1. Charge pump circuit.

  2. External 5V supply.

It's quite convenient to simply add a USB B connector for the 5V supply.

## Status

The protocol used between the N64 and controller, and between the controller
and Controller Pak is well understood now. An RS-232 to N64 bridge and several
Python programs demonstrate just about all of the protocol.

The necessary subset of the protocol has been implemented in the Cube64
firmware to emulate an official N64 controller with Rumble Pak. If 5V power
for the motor can be supplied to the GameCube controller, its rumble motor
will be used to emulate the N64 Rumble Pak.

It should be possible to emulate the N64 Controller Pak using the results of the
reverse engineering.

## Contents

##### firmware
Source code for the firmware, in PIC assembly. It was developed
using the open source "gputils" package. The firmware for the
adaptor itself is "cube64.asm", but there are a few additional
test programs and libraries.

##### hardware
This includes schematics for the adaptor, in PDF and gschem format.
##### notes

Utilities and documentation produced while reverse engineering.

## Authors
* **Micah Elizabeth Scott** - *Initial version* - [scanlime](https://github.com/scanlime)
* **Jacques Gagnon** - *Newer stuff* - [DarthCloud](https://github.com/darthcloud)

## References
* <http://www.int03.co.uk/crema/hardware/gamecube/gc-control.htm>
* <http://hitmen.c02.at/files/yagcd/yagcd/chap9.html>
* [http://instruct1.cit.cornell.edu/courses/ee476/FinalProjects/s2002/jew17/lld.html](https://web.archive.org/web/20130607051924/http://instruct1.cit.cornell.edu/courses/ee476/FinalProjects/s2002/jew17/lld.html#)
* [http://www.mixdown.ca/n64dev/](https://web.archive.org/web/20101230015154/http://www.mixdown.ca:80/n64dev/)

## External links
* [Original repository archive](https://github.com/scanlime/navi-misc/tree/master/wasabi/devices/cube64)
* [Google Code repository archive](https://code.google.com/archive/p/cube64-dx/)
* [Blog post](http://scanlime.org/2011/03/cube64-gamecube-to-n64-adaptor/)
