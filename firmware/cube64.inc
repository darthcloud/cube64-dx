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
    #define BIT_RUMBLE_MOTOR_ON         0x01
    #define FLAG_RUMBLE_MOTOR_ON        flags, 0, b

    ;; Flag for detecting when all buttons are released.
    #define BIT_NO_VIRTUAL_BTNS         0x04
    #define FLAG_NO_VIRTUAL_BTNS        flags, 2, b

    ;; Flag to force reporting emptied slot.
    #define BIT_FORCE_EMPTIED           0x08
    #define FLAG_FORCE_EMPTIED          flags, 3, b

    ;; Flag to control menu rumble feedback.
    #define BIT_RUMBLE_FEEDBACK         0x10
    #define FLAG_RUMBLE_FEEDBACK        flags, 4, b

    ;; Is the GC controller a WaveBird?
    #define BIT_CTRL_PENDING_INIT       0x20
    #define FLAG_CTRL_PENDING_INIT      flags, 5, b

    ;; WaveBird association state.
    #define BIT_CTRL_READY              0x40
    #define FLAG_CTRL_READY             flags, 6, b

    ;; Adaptor bypass mode.
    ;#define BIT_BYPASS_MODE             0x80
    ;#define FLAG_BYPASS_MODE            flags, 7, b

    ;; Flag for tracking if remap source is an axis.
    #define BIT_AXIS                    0x01
    #define FLAG_AXIS                   flags2, 0, b

    ;; Flag for button layout modification.
    #define BIT_LAYOUT_MODIFIER         0x02
    #define FLAG_LAYOUT_MODIFIER        flags2, 1, b

    ;; Flag for N64 CTRL2.
    #define BIT_CTRL2                   0x04
    #define FLAG_CTRL2                  flags2, 2, b

    ;; Set when we want to remap an analog trigger.
    #define FLAG_TRIGGER                flags2, 3, b

    ;; Joystick menu flags.
    #define FLAG_CS                     flags2, 4, b
    #define FLAG_AXIS_Y                 flags2, 5, b

    ;; Joystick calibrated.
    #define FLAG_CALIBRATED             flags2, 6, b

    ;; Set when we are waiting for item selection in the top config menu.
    #define BIT_TOP_CONFIG_MENU         0x01
    #define FLAG_TOP_CONFIG_MENU        menu_flags, 0, b

    ;; Set when we are waiting for user input in the adaptor mode submenu.
    #define BIT_MODE_SUBMENU            0x02
    #define FLAG_MODE_SUBMENU           menu_flags, 1, b

    ;; Set when we are waiting for user input in the button layout submenu.
    #define BIT_LAYOUT_SUBMENU          0x04
    #define FLAG_LAYOUT_SUBMENU         menu_flags, 2, b

    ;; Set when we're waiting for the source key for one of the combos.
    #define BIT_SOURCE_WAIT             0x08
    #define FLAG_SOURCE_WAIT            menu_flags, 3, b

    ;; Set when we're waiting for the destination key for one of the combos.
    #define BIT_REMAP                   0x10
    #define BIT_SPECIAL                 0x20
    #define BIT_JOYSTICK                0x40
    #define FLAG_REMAP                  menu_flags, 4, b
    #define FLAG_SPECIAL                menu_flags, 5, b
    #define FLAG_JOYSTICK               menu_flags, 6, b

    ;; We're waiting for a button to be released, cleared when no buttons are pressed.
    #define BIT_WAITING_FOR_RELEASE     0x80
    #define FLAG_WAITING_FOR_RELEASE    menu_flags, 7, b

    ;; Set when we want to disable GC joysticks scaling.
    #define NV_FLAG_SCALING_OFF         nv_flags, 2, b

    ;; Adaptor bypass mode.
    #define NV_FLAG_BYPASS_MODE         nv_flags, 5, b

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
        BTN_NONE:0
        CONFIG_JS
        CONFIG_CS
        LAYOUT_MAX
    endc

    ;; Adapter internal special button, do not map to any button on the host system.
    cblock 0x20
        BTN_MODIFIER:4
    endc

    #define CURVE_MASK          0x07
    #define CURVE_MASK_Y        0x70
    #define CURVE_BIT           2
    #define CURVE_BIT_Y         6
    #define SCALE_BIT           3
    #define SCALE_BIT_Y         7
    #define TRIGGER_TYPE_MASK   0xFD
    #define LAYOUT_MASK         0x03
    #define MODE_MASK           0x18
    #define BTN_MASK            0x1F
    #define SPECIAL_MASK        0x20
    #define SPECIAL_BIT         5

    #define AXIS_DEAD_ZONE      0x0A
    #define AXIS_BTN_THRS       0x30
    #define AXIS_BTN_VALUE      0x54
    #define TRIGGER_BTN_THRS    0x18

    ;; Number of bytes needed in EEPROM for each buttons configuration.
    #define EEPROM_BTN_BYTE     1

    ;; Buttons layout addresses.
    #define EEPROM_LAYOUT_SIZE  LAYOUT_MAX*EEPROM_BTN_BYTE
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
    movff   PRODL, temp3
    movf    layout, w, b                         ; Add offset to read in right buttons layout.
    mullw   EEPROM_LAYOUT_SIZE
    movf    PRODL, w, a
    addwf   temp3, w, b
    if byte
        addlw   byte
    endif
    endm

btfsc_config_js macro axis_byte, bit
    if axis_byte & 0x02
        if axis_byte & 0x01
            btfsc   nv_config_js, bit+4, b
        else
            btfsc   nv_config_js, bit, b
        endif
    else
        if axis_byte & 0x01
            btfsc   nv_config_cs, bit+4, b
        else
            btfsc   nv_config_cs, bit, b
        endif
    endif
    endm

btfss_config_js macro axis_byte, bit
    if axis_byte & 0x02
        if axis_byte & 0x01
            btfss   nv_config_js, bit+4, b
        else
            btfss   nv_config_js, bit, b
        endif
    else
        if axis_byte & 0x01
            btfss   nv_config_cs, bit+4, b
        else
            btfss   nv_config_cs, bit, b
        endif
    endif
    endm

movf_config_js macro axis_byte
    if axis_byte & 0x02
        if axis_byte & 0x01
            swapf   nv_config_js, w, b
        else
            movf    nv_config_js, w, b
        endif
    else
        if axis_byte & 0x01
            swapf   nv_config_cs, w, b
        else
            movf    nv_config_cs, w, b
        endif
    endif
    endm

