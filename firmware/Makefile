
all: cube64_16f84.hex cube64_12f629.hex serial_bridge.hex

cube64_16f84.hex: cube64.asm *.inc
	gpasm -p PIC16F84A -o $@ $<

cube64_12f629.hex: cube64.asm *.inc
	gpasm -p PIC12F629 -o $@ $<

%.hex: %.asm *.inc
	gpasm $<

install-84: cube64_16f84.hex
	pista < pista-84.conf

install-629: cube64_12f629.hex
	pista < pista-629.conf

clean:
	rm -f *.hex *.lst *.cod