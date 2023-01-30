blinky.bin: blinky.v blinky.pcf blinky.yosys
	yosys -s blinky.yosys
	nextpnr-ice40 --up5k --package sg48 --pcf blinky.pcf --json blinky.json --asc blinky.asc --pcf-allow-unconstrained
	icepack blinky.asc blinky.bin

clean:
	rm -f blinky.asc blinky.bin blinky.json

.PHONY: clean
