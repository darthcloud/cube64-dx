	;;
	;; Simple non-interrupt-driven software UART implemented using TMR0
	;; Copyright (C) 2004 Micah Dowty <micah@navi.cx>
	;;
	;;   This program is free software; you can redistribute it and/or modify
	;;   it under the terms of the GNU General Public License as published by
	;;   the Free Software Foundation; either version 2 of the License, or
	;;   (at your option) any later version.
	;;

	#define	POLARITY_NORMAL		0	; Connected using an RS-232 level shifted
	#define	POLARITY_INVERTED	1	; Connected directly to an RS-232 line

	;; Baud rate constants for 20 MHz
	#define B38400	.126
	#define B57600	.170
	#define B115200	.213


	;; *******************************************************************************
	;; **************************************************************** Receive ******
	;; *******************************************************************************

	;; Receives one byte into the given file register.
	;; The macro needs a port and bit for the pin to receive on,
	;; a speed selected from the #defines above, and a polarity
	;; also from the defines above.
	;;
	;; Requires a 'temp' register.
	;;
rs232_rx_byte	macro	pin_port, pin_bit, speed, polarity, reg
	local	wait_for_start
	local	wait_for_stop
	local	bit_loop

	banksel	OPTION_REG	; Run TMR0 from the instruction clock
	bcf	OPTION_REG, T0CS
	banksel	pin_port

wait_for_start			; Wait for a start bit to begin
	clrwdt
	if	polarity
	btfss	pin_port, pin_bit
	else
	btfsc	pin_port, pin_bit
	endif
	goto	wait_for_start

	clrf	TMR0		; Init the timer, fudge the initial
	bcf	INTCON, T0IF	; value so we end up in the middle of the first bit.
	movlw	speed - (.256 - speed)/3
	movwf	TMR0

	movlw	8		; Loop over 8 bits
	movwf	temp

bit_loop
	clrwdt
	btfss	INTCON, T0IF	; Wait for TMR0 to overflow
	goto	bit_loop

	clrw			; Sample the RS-232 line into our carry flag
	addlw	0x80		; (Put 0x80 in w, clear carry)
	if	polarity
	btfss	pin_port, pin_bit
	else
	btfsc	pin_port, pin_bit
	endif
	addlw	0x80		; Will always carry

	rrf	reg, f		; Shift in our new bit, LSB first

	bcf	INTCON, T0IF	; Clear the overflow
	movlw	speed		; Adjust TMR0's speed
	addwf	TMR0, f

	decfsz	temp, f		; Next bit...
	goto	bit_loop

wait_for_stop			; Now look for a stop bit...
	clrwdt
	btfss	INTCON, T0IF	; Wait for TMR0 to overflow
	goto	wait_for_stop

	if	polarity
	btfsc	pin_port, pin_bit
	else
	btfss	pin_port, pin_bit
	endif
	goto	wait_for_start	; Start all over again if there's no stop bit
	endm


	;; *******************************************************************************
	;; **************************************************************** Transmit *****
	;; *******************************************************************************

	;; Transmits one byte from the given register, whose value is not preserved.
	;; The macro needs a port and bit for the pin to receive on,
	;; a speed selected from the #defines above, and a polarity
	;; also from the defines above.
	;;
	;; Requires a 'temp' register.
	;;
rs232_tx_byte	macro	pin_port, pin_bit, speed, polarity, reg
	local	wait_for_start
	local	wait_for_stop
	local	wait_for_stop_done
	local	bit_loop
	local	output_zero
	local	output_done

	banksel	OPTION_REG	; Run TMR0 from the instruction clock
	bcf	OPTION_REG, T0CS
	banksel	pin_port

	if	polarity	; Begin a start bit
	bsf	pin_port, pin_bit
	else
	bcf	pin_port, pin_bit
	endif

	clrf	TMR0		; Init the timer
	bcf	INTCON, T0IF
	movlw	speed
	movwf	TMR0

	movlw	8		; Loop over 8 bits
	movwf	temp

bit_loop
	clrwdt
	btfss	INTCON, T0IF	; Wait for TMR0 to overflow
	goto	bit_loop

	rrf	reg, f		; Shift out a bit, LSB first
	btfss	STATUS, C
	goto	output_zero

	if	polarity	; Output a one
	bcf	pin_port, pin_bit
	else
	bsf	pin_port, pin_bit
	endif
	goto	output_done

output_zero
	if	polarity	; Output a zero
	bsf	pin_port, pin_bit
	else
	bcf	pin_port, pin_bit
	endif
output_done

	bcf	INTCON, T0IF	; Clear the overflow
	movlw	speed		; Adjust TMR0's speed
	addwf	TMR0, f

	decfsz	temp, f		; Next bit...
	goto	bit_loop

wait_for_stop			; Wait until it's time to start the stop bit
	clrwdt
	btfss	INTCON, T0IF
	goto	wait_for_stop

	if	polarity
	bcf	pin_port, pin_bit
	else
	bsf	pin_port, pin_bit
	endif

	bcf	INTCON, T0IF	; Clear the overflow
	movlw	speed		; Adjust TMR0's speed
	addwf	TMR0, f

wait_for_stop_done		; Don't return until the stop bit's done, for the next byte's sake
	clrwdt
	btfss	INTCON, T0IF
	goto	wait_for_stop_done

	endm

	;;; The End ;;;