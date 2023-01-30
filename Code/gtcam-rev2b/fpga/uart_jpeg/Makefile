COMMON:=../common

VERILOG_FILES:=
VERILOG_FILES+=$(COMMON)/util.v
VERILOG_FILES+=$(COMMON)/uart.v
VERILOG_FILES+=uart_jpeg_top.v
#VERILOG_FILES+=i2c_master.v
#VERILOG_FILES+=i2c_init.v
VERILOG_FILES+=uart_buffer.v
#VERILOG_FILES+=grayscale_downsampler.v

JFPJC_VERILOG:=../jfpjc/jfpjc/verilog
VERILOG_FILES+= $(JFPJC_VERILOG)/bitpacker.v
VERILOG_FILES+= $(JFPJC_VERILOG)/hm01b0_ingester.v
VERILOG_FILES+= $(JFPJC_VERILOG)/loeffler_dct_88.v
VERILOG_FILES+= $(JFPJC_VERILOG)/loeffler_dct_8.v
VERILOG_FILES+= $(JFPJC_VERILOG)/pipelined_multiplier.v
VERILOG_FILES+= $(JFPJC_VERILOG)/coefficient_encoder.v
VERILOG_FILES+= $(JFPJC_VERILOG)/jpeg_huffman_encode.v
VERILOG_FILES+= $(JFPJC_VERILOG)/hm01b0_sim.v
VERILOG_FILES+= $(JFPJC_VERILOG)/ice40_ebr.v
VERILOG_FILES+= $(JFPJC_VERILOG)/jfpjc.v
VERILOG_FILES+= $(JFPJC_VERILOG)/pipelined_divider.v
VERILOG_FILES+= $(JFPJC_VERILOG)/address_zigzagger.v
VERILOG_FILES+= $(JFPJC_VERILOG)/width_adapter_buffer.v
VERILOG_FILES+= $(JFPJC_VERILOG)/bytestuffer.v
VERILOG_FILES+= $(JFPJC_VERILOG)/dct_reset_manager.v
VERILOG_FILES+= $(JFPJC_VERILOG)/quantizer_manager_fsm.v
VERILOG_FILES+= $(JFPJC_VERILOG)/padding_trimmer.v

all: uart_jpeg.bin

uart_buffer_tb.vvp: ../common/util.v uart_buffer.v uart_buffer_tb.v
	iverilog -I../common $^ -Wall -o $@

uart_jpeg.bin: $(VERILOG_FILES) i2c_bytes.hex
	yosys -s uart_jpeg.yosys
	nextpnr-ice40 --up5k --package sg48 --pcf uart_jpeg.pcf --json uart_jpeg.json --asc uart_jpeg.asc --pcf-allow-unconstrained
	icepack uart_jpeg.asc uart_jpeg.bin

#uart_jpeg_tb.vvp: uart_jpeg.v uart_jpeg_tb.v ../common/util.v
#	iverilog $^ -Wall -o $@

clean:
	rm -f uart_jpeg.asc uart_jpeg.bin uart_jpeg.json uart_jpeg_tb.vvp uart_jpeg.vcd uart_buffer_tb.vvp

.PHONY: clean
