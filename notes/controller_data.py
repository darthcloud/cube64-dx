#!/usr/bin/env python3
"""Maximum values sampled from various controllers.

 --Jacques Gagnon <darthcloud@gmail.com>
"""

from collections import namedtuple

Point = namedtuple('Point', 'x y')
Origin = namedtuple('Origin', 'up right down left')
Diagonal = namedtuple('Diagonal', 'up_right down_right down_left up_left')
Maximum = namedtuple('Maximum', 'origin diagonal')

CTRL_DATA = {
    # Data sampled directly from N64 controller.
    "N64": [
        # 1996 NUS-005 Launch Controller (Solid Grey)
        # This controller joystick is quite beaten up as you can imagine.
        # Ambiguous bit sent to it (2 us low, 2 us high) are randomly interpreted as either 0 or 1.
        #[
        #    Maximum(
        #        Origin(Point(0, 0x47), Point(0x42, 0), Point(0, -0x50), Point(-0x47, 0)),
        #        Diagonal(Point(0x38, 0x39), Point(0x3c, -0x3f), Point(-0x38, -0x45), Point(-0x41, 0x40))
        #    ),
        #],

        # ~2002 NUS-005 Plastic Packaging Controller (Grape Purple)
        # Joystick is in excellent condition.
        # Older controllers will report controller slot status 0x00 (Empty) at power up.
        # This one report right away 0x02 (Emptied). Cube64 mimic this behavior when disabling
        # rumble emulation at boot up.
        # Ambiguous bit sent to it (2 us low, 2 us high) are always interpreted as 1.
        [
            Maximum(
                Origin(Point(0, 0x57), Point(0x52, 0), Point(0, -0x55), Point(-0x52, 0)),
                Diagonal(Point(0x47, 0x48), Point(0x48, -0x45), Point(-0x43, -0x46), Point(-0x48, 0x49))
            ),
        ],

        # 2003 iQue Controller
        # Brand-new
        # Internally a regular N64 controller. No slot externally.
        #[
        #    Maximum(
        #        Origin(Point(0, 0x53), Point(0x54, 0), Point(0, -0x57), Point(-0x58, 0)),
        #        Diagonal(Point(0x45, 0x42), Point(0x44, -0x44), Point(-0x4a, -0x4a), Point(-0x4a, 0x41))
        #    ),
        #],

        # Horipad Mini 64
        # Brand-new
        # Joystick do not respect regular controller range.
        # The bit period is 6 us!
        #[
        #    Maximum(
        #        Origin(Point(0, 0x57), Point(0x70, 0), Point(0, -0x66), Point(-0x5e, 0)),
        #        Diagonal(Point(0x56, 0x46), Point(0x59, -0x4e), Point(-0x3f, -0x51), Point(-0x41, 0x45))
        #    ),
        #],
    ],

    # Data sampled through the Cube64 adaptor without any scaling.
    # Deadzone and sign was applied.
    "GC": [
        # DOL-003 Controller (Silver)
        # Joystick is in excellent condition.
        [
            Maximum(
                Origin(Point(0, 0x5b), Point(0x5a, 0), Point(0, -0x5b), Point(-0x5b, 0)),
                Diagonal(Point(0x45, 0x43), Point(0x44, -0x41), Point(-0x42, -0x3c), Point(-0x40, 0x42))
            ),
            Maximum(
                Origin(Point(0, 0x53), Point(0x54, 0), Point(0, -0x55), Point(-0x53, 0)),
                Diagonal(Point(0x40, 0x3a), Point(0x3e, -0x3e), Point(-0x3d, -0x37), Point(-0x39, 0x3b))
            ),
        ],

        # DOL-003 Controller SSBB Wii (White)
        # Brand-new.
        [
            Maximum(
                Origin(Point(0, 0x5a), Point(0x59, 0), Point(0, -0x57), Point(-0x53, 0)),
                Diagonal(Point(0x42, 0x41), Point(0x41, -0x46), Point(-0x3a, -0x43), Point(-0x3a, 0x41))
            ),
            Maximum(
                Origin(Point(0, 0x4e), Point(0x51, 0), Point(0, -0x5c), Point(-0x50, 0)),
                Diagonal(Point(0x3b, 0x35), Point(0x3a, -0x44), Point(-0x38, -0x3e), Point(-0x35, 0x36))
            ),
        ],

        # DOL-003 Controller SSBB Wii (White)
        # Brand-new.
        [
            Maximum(
                Origin(Point(0, 0x59), Point(0x55, 0), Point(0, -0x5e), Point(-0x59, 0)),
                Diagonal(Point(0x3c, 0x42), Point(0x3c, -0x46), Point(-0x3f, -0x42), Point(-0x3f, 0x40))
            ),
            Maximum(
                Origin(Point(0, 0x51), Point(0x4c, 0), Point(0, -0x5e), Point(-0x55, 0)),
                Diagonal(Point(0x35, 0x39), Point(0x35, -0x41), Point(-0x3d, -0x3d), Point(-0x3b, 0x39))
            ),
        ],

        # DOL-003 Controller SSB WiiU (White)
        # Brand-new.
        [
            Maximum(
                Origin(Point(0, 0x5d), Point(0x5a, 0), Point(0, -0x5a), Point(-0x59, 0)),
                Diagonal(Point(0x41, 0x43), Point(0x41, -0x3f), Point(-0x41, -0x3c), Point(-0x40, 0x41))
            ),
            Maximum(
                Origin(Point(0, 0x50), Point(0x52, 0), Point(0, -0x5b), Point(-0x57, 0)),
                Diagonal(Point(0x3c, 0x37), Point(0x3b, -0x43), Point(-0x41, -0x3c), Point(-0x3e, 0x37))
            ),
        ],

        # WaveBird Controller (Silver)
        # Joystick is in excellent condition.
        [
            Maximum(
                Origin(Point(0, 0x5f), Point(0x58, 0), Point(0, -0x5d), Point(-0x59, 0)),
                Diagonal(Point(0x3e, 0x43), Point(0x3f, -0x42), Point(-0x3e, -0x40), Point(-0x40, 0x43))
            ),
            Maximum(
                Origin(Point(0, 0x57), Point(0x52, 0), Point(0, -0x4e), Point(-0x4d, 0)),
                Diagonal(Point(0x38, 0x3d), Point(0x38, -0x35), Point(-0x35, -0x36), Point(-0x35, 0x3a))
            ),
        ],

        # WaveBird Controller (Silver)
        # Joystick is in excellent condition.
        [
            Maximum(
                Origin(Point(0, 0x5e), Point(0x57, 0), Point(0, -0x56), Point(-0x5a, 0)),
                Diagonal(Point(0x3c, 0x44), Point(0x3d, -0x3c), Point(-0x4d, -0x3c), Point(-0x43, 0x45))
            ),
            Maximum(
                Origin(Point(0, 0x52), Point(0x4a, 0), Point(0, -0x4e), Point(-0x56, 0)),
                Diagonal(Point(0x32, 0x38), Point(0x33, -0x36), Point(-0x3f, -0x35), Point(-0x3c, 0x39))
            ),
        ],

        # WaveBird Controller (Grey)
        # Joystick is in excellent condition.
        [
            Maximum(
                Origin(Point(0, 0x64), Point(0x5b, 0), Point(0, -0x59), Point(-0x60, 0)),
                Diagonal(Point(0x40, 0x49), Point(0x42, -0x3e), Point(-0x46, -0x3c), Point(-0x4a, 0x4c))
            ),
            Maximum(
                Origin(Point(0, 0x4f), Point(0x4f, 0), Point(0, -0x56), Point(-0x57, 0)),
                Diagonal(Point(0x35, 0x34), Point(0x37, -0x3a), Point(-0x3f, -0x38), Point(-0x3e, 0x37))
            ),
        ],

        # Sammy Keyboard Controller
        # Brand-new.
        #[
        #    Maximum(
        #        Origin(Point(0, 0x5d), Point(0x5c, 0), Point(0, -0x5b), Point(-0x5e, 0)),
        #        Diagonal(Point(0x43, 0x41), Point(0x41, -0x3e), Point(-0x42, -0x3d), Point(-0x47, 0x43))
        #    ),
        #    Maximum(
        #        Origin(Point(0, 0x4e), Point(0x49, 0), Point(0, -0x50), Point(-0x5a, 0)),
        #        Diagonal(Point(0x33, 0x39), Point(0x34, -0x35), Point(-0x3c, -0x37), Point(-0x40, 0x34))
        #    ),
        #],

        # GC DS5 3/4 Speed Prototype Controller
        # Joystick is in excellent condition.
        # The bit period is 5.3 us (3/4 of regular frequency)
        #[
        #    Maximum(
        #        Origin(Point(0, 0x55), Point(0x55, 0), Point(0, -0x56), Point(-0x58, 0)),
        #        Diagonal(Point(0x38, 0x38), Point(0x3a, -0x3a), Point(-0x41, -0x42), Point(-0x3b, 0x3a))
        #    ),
        #    Maximum(
        #        Origin(Point(0, 0x51), Point(0x5a, 0), Point(0, -0x6f), Point(-0x5d, 0)),
        #        Diagonal(Point(0x3b, 0x35), Point(0x42, -0x50), Point(-0x43, -0x58), Point(-0x3f, 0x32))
        #    ),
        #],
    ]
}

### The End ###
