all: uart_lfsr.bin uart_lfsr_tb.vvp

uart_lfsr.bin: uart_lfsr.v uart_lfsr.pcf uart_lfsr.yosys
	yosys -s uart_lfsr.yosys
	nextpnr-ice40 --up5k --package sg48 --pcf uart_lfsr.pcf --json uart_lfsr.json --asc uart_lfsr.asc --pcf-allow-unconstrained
	icepack uart_lfsr.asc uart_lfsr.bin

uart_lfsr_tb.vvp: uart_lfsr.v uart_lfsr_tb.v ../common/util.v
	iverilog $^ -Wall -o $@

clean:
	rm -f uart_lfsr.asc uart_lfsr.bin uart_lfsr.json uart_lfsr_tb.vvp uart_lfsr.vcd

.PHONY: clean
