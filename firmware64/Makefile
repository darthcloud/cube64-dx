
all: cube64-dx_18f14k22.hex

cube64-dx_18f14k22.hex: cube64-dx.asm *.inc
	gpasm -p PIC18F14K22 -o $@ $<

%.hex: %.asm *.inc
	gpasm $<
	
clean:
	rm -f *.hex *.lst *.cod