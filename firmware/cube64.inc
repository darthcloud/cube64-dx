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
    #define BIT_NO_VIRTUAL_BTNS         0x02
    #define FLAG_NO_VIRTUAL_BTNS        flags, 1, b

    ;; Flag to force reporting emptied slot.
    #define BIT_FORCE_EMPTIED           0x04
    #define FLAG_FORCE_EMPTIED          flags, 2, b

    ;; Flag to control menu rumble feedback.
    #define BIT_RUMBLE_FEEDBACK         0x08
    #define FLAG_RUMBLE_FEEDBACK        flags, 3, b

    ;; Is the GC controller a WaveBird?
    #define BIT_CTRL_PENDING_INIT       0x10
    #define FLAG_CTRL_PENDING_INIT      flags, 4, b

    ;; WaveBird association state.
    #define BIT_CTRL_READY              0x20
    #define FLAG_CTRL_READY             flags, 5, b

    ;; Adaptor bypass mode.
    #define BIT_BYPASS_MODE             0x40
    #define FLAG_BYPASS_MODE            flags, 6, b

    ;; Joystick calibrated.
    #define BIT_CALIBRATED              0x80
    #define FLAG_CALIBRATED             flags, 7, b

    ;; Flag for tracking if remap source is an axis.
    #define FLAG_AXIS                   flags2, 0, b

    ;; Flag for button layout modification.
    #define FLAG_LAYOUT_MODIFIER        flags2, 1, b

    ;; Flag for N64 CTRL2.
    #define FLAG_CTRL2                  flags2, 2, b

    ;; Set when we want to remap an analog trigger.
    #define FLAG_TRIGGER                flags2, 3, b

    ;; Joystick menu flags.
    #define FLAG_CS                     flags2, 4, b
    #define FLAG_AXIS_Y                 flags2, 5, b

    ;; Menu multi-level option flags.
    #define OFLAG_MODE                   option_flags, 0, b
    #define OFLAG_LAYOUT                 option_flags, 1, b
    #define OFLAG_JOYSTICK               option_flags, 2, b
    #define OFLAG_PRESET                 option_flags, 3, b
    #define OFLAG_REMAP                  option_flags, 4, b
    #define OFLAG_SPECIAL                option_flags, 5, b

    ;; We're waiting for a button to be released, cleared when no buttons are pressed.
    #define UFLAG_WAITING_FOR_RELEASE    uncommit_flags, 0 ,b
    #define AFLAG_WAITING_FOR_RELEASE    atomic_flags, 0, b

    ;; Menu levels flags.
    #define UFLAG_MENU_LEVEL1            uncommit_flags, 1, b
    #define UFLAG_MENU_LEVEL2            uncommit_flags, 2, b
    #define UFLAG_MENU_LEVEL3            uncommit_flags, 3 ,b
    #define AFLAG_MENU_LEVEL1            atomic_flags, 1, b
    #define AFLAG_MENU_LEVEL2            atomic_flags, 2, b
    #define AFLAG_MENU_LEVEL3            atomic_flags, 3 ,b

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
        BTN_FUTURE:.18
        BTN_A_C_DOWN
        BTN_B_C_LEFT
    endc

    #define CURVE_MASK          0x07
    #define CURVE_MASK_Y        0x70
    #define CURVE_BIT           2
    #define CURVE_BIT_Y         6
    #define SCALE_BIT           3
    #define SCALE_BIT_Y         7
    #define TRIGGER_TYPE_MASK   0xFD
    #define LAYOUT_MASK         0x03
    #define PRESET_MASK         0x07
    #define BTN_MASK            0x1F
    #define SPECIAL_MASK        0x20
    #define SPECIAL_BIT         5
    #define TURBO_MASK          0x03

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
    andlw   LAYOUT_MASK
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

set_interrupt macro
    movf    PORTA, w, a
    movf    PORTB, w, a
    nop
    bcf     INTCON, RABIF, a
    bsf     INTCON, GIEH, a
    endm

clr_interrupt macro
    bcf     INTCON, GIEH, a
    endm

