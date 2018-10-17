
all: cube64_diy_k22.hex

q10: cube64_smd_q10.hex

cube64_diy_k22.hex: cube64.asm *.inc
	gpasm -y -S 2 -p PIC18F14K22 -o $@ $<

cube64_smd_q10.hex: cube64.asm *.inc
	gpasm -y -S 2 -p PIC18F24Q10 -o $@ $<

serial_bridge_k22.hex: serial_bridge.asm *.inc
	gpasm -y -S 2 -p PIC18F14K22 -o $@ $<

serial_bridge_q10.hex: serial_bridge.asm *.inc
	gpasm -y -S 2 -p PIC18F24Q10 -o $@ $<

install: cube64_diy_k22.hex
	ipecmd -TPPK4 -P18F14K22 -M -F$<

install_q10: cube64_smd_q10.hex
	ipecmd -TPPK4 -P18F24Q10 -M -F$<

install_bridge: serial_bridge_k22.hex
	ipecmd -TPPK4 -P18F14K22 -M -F$<

install_bridge_q10: serial_bridge_q10.hex
	ipecmd -TPPK4 -P18F24Q10 -M -F$<

clean:
	rm -f *.hex *.lst *.cod log* *.xml*
