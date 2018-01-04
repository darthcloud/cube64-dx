
all: cube64-dx_18f14k22.hex

cube64-dx_18f14k22.hex: cube64.asm *.inc
	gpasm -p PIC18F14K22 -o $@ $<

install: cube64-dx_18f14k22.hex
	pk2cmd -PPIC18F14K22 -M -F $<

%.hex: %.asm *.inc
	gpasm $<

clean:
	rm -f *.hex *.lst *.cod
