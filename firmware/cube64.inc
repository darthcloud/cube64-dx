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

    ;; Flag for tracking if remap source is an axis.
    #define FLAG_AXIS                   flags, 3

    ;; Flag for button layout modification.
    #define FLAG_LAYOUT_MODIFIER        flags, 4

    ;; Is the GC controller a WaveBird?
    #define WAVEBIRD                    flags, 5

    ;; WaveBird association state.
    #define WAVEBIRD_ASSOCIATED         flags, 6

    ;; Adaptor bypass mode.
    #define FLAG_BYPASS_MODE            flags, 7

    ;; Flag to force reporting emptied slot.
    #define FLAG_FORCE_EMPTIED          flags2, 0

    ;; Flag to control menu rumble feedback.
    #define FLAG_RUMBLE_FEEDBACK        flags2, 1

    ;; Set when we are waiting for item selection in the top config menu.
    #define FLAG_TOP_CONFIG_MENU        menu_flags, 0

    ;; Set when we are waiting for user input in the adaptor mode submenu.
    #define FLAG_MODE_SUBMENU           menu_flags, 1

    ;; Set when we are waiting for user input in the button layout submenu.
    #define FLAG_LAYOUT_SUBMENU         menu_flags, 2

    ;; Set when we're waiting for the source key for one of the combos.
    #define FLAG_SOURCE_WAIT            menu_flags, 3

    ;; Set when we're waiting for the destination key for one of the combos.
    #define FLAG_REMAP                  menu_flags, 4
    #define FLAG_SPECIAL                menu_flags, 5

    ;; Set when we want to remap an analog trigger.
    #define FLAG_TRIGGER                menu_flags, 6

    ;; Set when we want to disable GC joysticks scaling.
    #define NV_FLAG_SCALING_OFF         nv_flags, 2

    ;; Button IDs. These are a superset of the GameCube and N64, without any correspondence
    ;; with the wire protocol used by either. They're used as intermediate values when
    ;; translating from GameCube to N64. These IDs are used as indices into the remapping
    ;; table stored in our on-chip EEPROM.

    cblock 0x00
        ;; Setup directional values so that 2 LSB are: up=0, left=1, right=2, down=3.
        ;; Also positive direction (up, right) has LSB clear while negative direction
        ;; (left, down) has LSB set.
        BTN_D_UP
        BTN_D_LEFT
        BTN_D_RIGHT
        BTN_D_DOWN
        BTN_LJ_UP
        BTN_LJ_LEFT
        BTN_LJ_RIGHT
        BTN_LJ_DOWN
        BTN_RJ_UP
        BTN_RJ_LEFT
        BTN_RJ_RIGHT
        BTN_RJ_DOWN

        ;; Setup analog triggers so their LSB is clear since they are positive axes
        ;; and make sure both analog trigger has only 1 bit different (2nd bit).
        BTN_LA
        BTN_L
        BTN_RA
        BTN_R
        BTN_LZ
        BTN_LG
        BTN_LJ
        BTN_RZ
        BTN_RG
        BTN_RJ

        BTN_A
        BTN_B
        BTN_X
        BTN_Y

        BTN_SELECT
        BTN_HOME
        BTN_START
        BTN_C
        BTN_NONE
    endc

    ;; Adapter internal special button, do not map to any button on the host system.
    cblock 0x20
        BTN_MODIFIER:4
    endc

    #define TRIGGER_TYPE_MASK   0xFD
    #define LAYOUT_MASK         0x03
    #define BTN_MASK            0x1F
    #define SPECIAL_MASK        0x20
    #define SPECIAL_BIT         5

    #define AXIS_DEAD_ZONE      0x0A
    #define AXIS_BTN_THRS       0x30
    #define AXIS_BTN_VALUE      0x54
    #define TRIGGER_BTN_THRS    0x18

    ;; Number of bytes needed in EEPROM for each buttons configuration.
    #define EEPROM_BTN_BYTE     2

    ;; Buttons layout addresses.
    #define EEPROM_LAYOUT_SIZE  BTN_NONE*EEPROM_BTN_BYTE
    #define EEPROM_LAYOUT_0     0x00
    #define EEPROM_LAYOUT_1     EEPROM_LAYOUT_0 + EEPROM_LAYOUT_SIZE
    #define EEPROM_LAYOUT_2     EEPROM_LAYOUT_1 + EEPROM_LAYOUT_SIZE
    #define EEPROM_LAYOUT_3     EEPROM_LAYOUT_2 + EEPROM_LAYOUT_SIZE
    #define EEPROM_NV_FLAGS     EEPROM_LAYOUT_3 + EEPROM_LAYOUT_SIZE

    ;; Magic word and the address it should be in the EEPROM,
    ;; as a big-endian 16-bit value.
    ;;
    ;; This is used to identify the contents of our EEPROM as ours,
    ;; so that if this firmware is installed on a chip with a blank
    ;; EEPROM or one with different data in it, we reinitialize it.
    ;; Change this value if the EEPROM data format changes.
    ;;

    #define EEPROM_MAGIC_WORD   0xEA5C
    #define EEPROM_MAGIC_ADDR   EEPROM_NV_FLAGS + 1

    ;; Convert virtual button in w to eeprom byte address in w.
eeprom_btn_addr macro layout, byte
    mullw   EEPROM_BTN_BYTE                      ; Offset base on how many bytes per button.
    movff   PRODL, temp
    movf    layout, w                            ; Add offset to read in right buttons layout.
    mullw   EEPROM_LAYOUT_SIZE
    movf    PRODL, w
    addwf   temp, w
    if byte
        addlw   byte
    endif
    endm

