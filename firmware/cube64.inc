    ;;
    ;; Definitions for Cube64 adaptor
    ;; Copyright (C) 2004 Micah Dowty <micah@navi.cx>
    ;;               2017 Jacques Gagnon <darthcloud@gmail.com>
    ;;
    ;;   This program is free software; you can redistribute it and/or modify
    ;;   it under the terms of the GNU General Public License as published by
    ;;   the Free Software Foundation; either version 2 of the License, or
    ;;   (at your option) any later version.
    ;;

    ;; The rumble motor should be on.
    #define FLAG_RUMBLE_MOTOR_ON        flags, 0

    ;; We're waiting for a button to be released, cleared when no buttons are pressed.
    #define FLAG_WAITING_FOR_RELEASE    flags, 1

    ;; Flag for detecting when all buttons are released.
    #define FLAG_NO_VIRTUAL_BTNS        flags, 2

    ;; Is the GC controller a WaveBird?
    #define WAVEBIRD                    flags, 3

    ;; WaveBird association state.
    #define WAVEBIRD_ASSOCIATED         flags, 4

    ;; Set when we are waiting for item selection in the top config menu.
    #define FLAG_TOP_CONFIG_MENU        config_flags, 0

    ;; Set when we are waiting for user input in the adaptor mode submenu.
    #define FLAG_MODE_SUBMENU           config_flags, 1

    ;; Set when we are waiting for user input in the button layout submenu.
    #define FLAG_LAYOUT_SUBMENU         config_flags, 2

    ;; Set when we're waiting for the source and destination keys, respectively, in a remap combo.
    #define FLAG_REMAP_SOURCE_WAIT      config_flags, 6
    #define FLAG_REMAP_DEST_WAIT        config_flags, 7

    ;; Button IDs. These are a superset of the GameCube and N64, without any correspondence
    ;; with the wire protocol used by either. They're used as intermediate values when
    ;; translating from GameCube to N64. These IDs are used as indices into the remapping
    ;; table stored in our on-chip EEPROM.

    #define BTN_A       0x00
    #define BTN_B       0x01
    #define BTN_X       0x02
    #define BTN_Y       0x03
    #define BTN_Z       0x04
    #define BTN_R       0x05
    #define BTN_L       0x06
    #define BTN_START   0x07
    #define BTN_D_LEFT  0x08
    #define BTN_D_RIGHT 0x09
    #define BTN_D_DOWN  0x0A
    #define BTN_D_UP    0x0B
    #define BTN_C_LEFT  0x0C
    #define BTN_C_RIGHT 0x0D
    #define BTN_C_DOWN  0x0E
    #define BTN_C_UP    0x0F

    #define AXIS_DEAD_ZONE      0x0A
    #define AXIS_BTN_THRS       0x30

    ;; Buttons layout addresses.
    #define EEPROM_LAYOUT_SIZE  0x10
    #define EEPROM_LAYOUT_0     0x00
    #define EEPROM_LAYOUT_1     EEPROM_LAYOUT_0 + EEPROM_LAYOUT_SIZE
    #define EEPROM_LAYOUT_2     EEPROM_LAYOUT_1 + EEPROM_LAYOUT_SIZE
    #define EEPROM_LAYOUT_3     EEPROM_LAYOUT_2 + EEPROM_LAYOUT_SIZE
    #define EEPROM_LAST_KEY_MAP EEPROM_LAYOUT_3 + EEPROM_LAYOUT_SIZE

    ;; Magic word and the address it should be in the EEPROM,
    ;; as a big-endian 16-bit value.
    ;;
    ;; This is used to identify the contents of our EEPROM as ours,
    ;; so that if this firmware is installed on a chip with a blank
    ;; EEPROM or one with different data in it, we reinitialize it.
    ;; Change this value if the EEPROM data format changes.
    ;;

    #define EEPROM_MAGIC_WORD   0xAC0F
    #define EEPROM_MAGIC_ADDR   EEPROM_LAST_KEY_MAP + 1

