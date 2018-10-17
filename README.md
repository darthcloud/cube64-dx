# Cube64

<p align="center"><img src=hardware/cube64_smd.png /></p>

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
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_js.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_js.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_d.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_d.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_a.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_a.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_b.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_b.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_z.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cright.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_start.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_start.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_dr.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_r.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_dl.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_z.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_cs.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_c.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_x.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cdown.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_y.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cleft.png)

By default the adaptor layout banks are loaded with the first four preset.
See [wiki](https://github.com/darthcloud/cube64-dx/wiki) for more information on the preset.

### Main menu

All the configurable feature of the adaptor can be accessed thought this menu.
To access the menu:

   1. Hold down the analog L and R buttons down most of the way, but not enough to
      push the end-stop buttons, and press Start.

The adaptor LED will be ON as long you are inside the menu.

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
     according to the table below.

  5. If the mapping was successful and you have rumble power connected, you should feel
     a brief rumble.

GameCube | N64
-------- | ---
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_js.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_js.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_d.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_d.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_a.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_a.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_b.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_b.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_z.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_z.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_start.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_start.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_dr.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_r.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_dl.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_l.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_cs.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_c.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_x.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/none.png)
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_y.png) | ![](https://github.com/darthcloud/cube64-dx/wiki/img/none.png)

#### Special function

It's possible to map a special function to a button instead of a N64 button.

  1. While in the main menu, press Y.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press the button you wish to set the function to.

  4. If you have the rumble power connected, you should feel a brief rumble.

  5. Press the button corresponding to the special function you want. This button is always
     interpreted according to the defaults below.

  6. If the mapping was successful and you have rumble power connected, you should feel
     a brief rumble.

##### Function available:

GameCube | Function
-------- | --------
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_d.png) | Temporary overwrite current mapping layout <br/>with layout corresponding to the D-pad <br/>direction until button is released.
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_a.png) | A+C-Down two buttons mapping.
![](https://github.com/darthcloud/cube64-dx/wiki/img/gc_b.png) | B+C-Right two buttons mapping.
![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cup.png) | Turbo disable (Default).
![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cleft.png) | Faster turbo.
![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cright.png) | Medium turbo.
![](https://github.com/darthcloud/cube64-dx/wiki/img/n64_cdown.png) | Slower turbo.

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

#### GameCube joysticks config

By default the joysticks value from the GameCube controller are scale to match the N64
joystick value range. This can be disabled to use the GameCube range instead (< v3.1 behavior).
A response curve can also be applied to a GC axis. By default none is applied (Linear).
Scale and curve setting are save independently for each axis and each layout.

  1. While in the main menu, press D-pad RIGHT.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press either direction on one of the four GC joystick axes.

  4. If you have the rumble power connected, you should feel a brief rumble.

  5. Press X to enable or Y to disable the scaling function.
     Or press one of the D-pad directions for enabling a response curve.
     (UP: Aggressive, LEFT: Relaxed, RIGHT: Wide, DOWN: S-Curve)
     Or press A for no response curve (Linear).

  6. If you have the rumble power connected and the layout change was successful, you should
     feel a brief rumble.

This option is saved into the EEPROM so the next time you power up the adaptor the last
used setting will be reloaded automatically.

#### Preconfigured button mapping layout

Eight pre-defined button mapping layout can be loaded in the currently active layout.
See [wiki](https://github.com/darthcloud/cube64-dx/wiki) for preset detail.

  1. While in the main menu, press D-pad DOWN.

  2. If you have the rumble power connected, you should feel a brief rumble.

  3. Press any direction on GC D-pad or main joystick to choose one preset.

  4. If you have the rumble power connected, you should feel a brief rumble.

### Calibration

The adaptor handles calibration in almost the exact same way a GameCube does. If
your joystick is drifting or unresponsive during gameplay, you should be able to
recalibrate it by performing any of the following actions without touching the
joystick, C-stick, L button, or R button:

  1. Hold down X, Y, and Start for a couple seconds during gameplay.

  2. Unplug the controller from the adaptor and plug it back in.

  3. Unplug the adaptor from the N64 and plug it back in.

  4. Turn off the N64 then turn it back on. (The reset button won't
     recalibrate, since the adaptor doesn't lose power or get reinitialized)

### 1 player 2 controllers control style

If connected to two ports of the system, the adaptor will answer automatically
a limited button status and identity commands on the 2nd port. This allows
to play GoldenEye 007 and Perfect Dark using their respective 1 player 2 controller
control style.

Analog input map for the 1st controller C Buttons will map to the 2nd controller joystick.
Digital input map for the 1st controller R button will map to the 2nd controller Z button.

## Hardware

There are currently two versions of the Cube64 hardware:

  - The "Cube64 DIY" uses a PIC18F14K22 microcontroller. Just about any
    electronics hobbyist should be able to build it easily. 5V can be
    provided either externally or by using a charge pump in 8-pin SOIC
    package.

  - The "Cube64 SMD" uses a PIC18F24Q10 microcontroller, and a
    charge pump in 10-pin DFN package. PCB gerber and KiCad sources
    are provided.

Both versions of the hardware have the same button remapping features
and responsiveness, as the only difference in their firmware is a few lines
worth of compile-time configuration. The DIP version of the Q10 could also
be used for the DIY version, but the package is bigger.

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
using the open source "gputils" package (use latest [snapshot](https://sourceforge.net/projects/gputils/files/snapshot_builds/src/)). The firmware for
the adaptor itself is "cube64.asm", but there are a few additional
test programs and libraries.

##### hardware
This includes schematics and PCB for the adaptor, in KiCad source format.

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

## Buttons graphic sources
* N64: [Toomai](https://www.ssbwiki.com/User:Toomai) @[ssbwiki.com](https://www.ssbwiki.com/User:Toomai/SVG_test_page)
* GC: [Flark](https://wiibrew.org/wiki/User:Flark) & [Crayon](https://wiibrew.org/wiki/User:Crayon) @[wiibrew.org](https://wiibrew.org/wiki/Category:Controller_Buttons)
* SVGs: Clone https://github.com/darthcloud/cube64-dx.wiki.git
