COMMON:=../../common

all: hm01b0_jpeg.bin

VERILOG_FILES:=
VERILOG_FILES+=$(COMMON)/util.v
VERILOG_FILES+=$(COMMON)/uart.v
VERILOG_FILES+=hm01b0_jpeg_top.v
VERILOG_FILES+=i2c_master.v
VERILOG_FILES+=i2c_init.v
VERILOG_FILES+=uart_buffer.v
VERILOG_FILES+=grayscale_downsampler.v

JFPJC_VERILOG:=./jfpjc/jfpjc/verilog
VERILOG_FILES+= $(JFPJC_VERILOG)/fixed_point_consts.v
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

iverilog: $(VERILOG_FILES) i2c_init_bytes.hex
	iverilog -Wall -Wno-sensitivity-entire-array -g2012 -o $@ $(filter %.v,$^)

iverilog2: $(VERILOG_FILES) i2c_init_bytes.hex
	iverilog -Wall -Wno-sensitivity-entire-array -g2012 hm01b0_jpeg_sim.v $(COMMON)/util.v test.sv

hm01b0_jpeg.bin: $(VERILOG_FILES) hm01b0_jpeg.pcf hm01b0_jpeg.yosys i2c_init_bytes.hex
	yosys -s hm01b0_jpeg.yosys
	nextpnr-ice40 --placer-budgets --no-tmdriv --up5k --package sg48 --pcf hm01b0_jpeg.pcf --json hm01b0_jpeg.json --asc hm01b0_jpeg.asc
	icepack hm01b0_jpeg.asc hm01b0_jpeg.bin

i2c_sender_tb.vvp: $(COMMON)/util.v i2c_sender_tb.v i2c_master.v i2c_init.v
	iverilog $^ -Wall -o $@

clean:
	rm hm01b0_jpeg.bin hm01b0_jpeg.asc hm01b0_jpeg.json i2c_sender_tb.vvp

.PHONY: clean
