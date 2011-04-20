
all: cube64_12f683.hex serial_bridge.hex

cube64_12f683.hex: cube64.asm *.inc
	gpasm -p PIC12F683 -o $@ $<

%.hex: %.asm *.inc
	gpasm $<
	
install-683: cube64_12f683.hex
	pista < pista-683.conf
	
clean:
	rm -f *.hex *.lst *.cod