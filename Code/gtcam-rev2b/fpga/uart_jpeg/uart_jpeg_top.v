`timescale 1ns/100ps

/**
 * Puts some data in at the end of a frame delimited by a vsync signal.
 *
 * Incoming vsync is active high.
 */
module frame_end_stuffer(input                      clock,
                         input                      reset,

                         input                      data_in_valid,
                         input                      vsync_in,
                         input [7:0]                data_in,

                         output reg state,
                         output reg                 data_out_valid,
                         output reg [7:0]           data_out,
                         output reg                 vsync_out);
    parameter delimiter_length = 32;   // should be a multiple of 8.
    parameter [(delimiter_length - 1):0] delimiter = 32'hf00f_ba11;
    localparam delimiter_index_maxvalue = (delimiter_length / 8) - 1;

    reg vsync_prev [0:1];

    always @* begin
        vsync_prev[0] = vsync_in;
    end

    localparam STATE_IMAGE = 1'b0;
    localparam STATE_DELIMITER = 1'b1;
    //reg state;
    reg [($clog2(delimiter_index_maxvalue) - 1):0] delimiter_index;
    always @(posedge clock) begin
        if (reset) begin
            data_out_valid <= 1'b0;
            dat_out <= 'hxx;
            vsync_out <= 1'b0;
            vsync_prev[1] <= 1'b0;
            state <= STATE_IMAGE;
            delimiter_index <= 'hxx;
        end else begin
            vsync_prev[1] <= vsync_prev[0];

            case (state)
                STATE_IMAGE: begin
                    data_out_valid <= data_in_valid;
                    data_out <= data_in;

                    if (!vsync_prev[0] && vsync_prev[1]) begin
                        vsync_out <= 1'b1;
                        state <= STATE_DELIMITER;
                    end else begin
                        vsync_out <= vsync_in;
                        state <= state;
                    end
                    delimiter_index <= 'h0;

                end

                STATE_DELIMITER: begin
                    data_out_valid <= 1'b1;
                    data_out <= delimiter[(((delimiter_length / 8) - delimiter_index - 1) * 8) +: 8];

                    if (delimiter_index == delimiter_index_maxvalue) begin
                        delimiter_index <= 'h0;
                        state <= STATE_IMAGE;
                        vsync_out <= 1'b1;
                    end else begin
                        delimiter_index <= delimiter_index + 'h1;
                        state <= state;
                        vsync_out <= 1'b1;
                    end
                end
            endcase
        end
    end
endmodule

`define DEV_ID 8'h24
/**
 *
 * outputs:
 *     active     - This signal is high as long as the i2c initializer is sending data. It goes low
 *                  once all data has been sent.
 */
module i2c_initializer(input clock, //should be 12mhz
                       input reset,

                       inout hm01b0_sda,
                       inout hm01b0_scl,

                       output reg active);

    reg sda,scl;
    assign hm01b0_sda = sda ? 1'bz : 1'b0;
    assign hm01b0_scl = scl ? 1'bz : 1'b0;

    wire scl_freq;
    //A11 = 12000000/(440*(2^7)) = 213.068
    //defparam div.N = 213; //120 for 100khz, 30 for 400khz
    //defparam div.N = 30;
    defparam div.N = 10;
    divide_by_n div(
        .clk(clock),
        .reset(reset),
        .out(scl_freq)
    );

    reg [23:0] mem [0:159];
    //integer i;
    initial begin
        $readmemh("i2c_bytes.hex", mem);
    end

    reg [7:0] mem_adr; //2^8=256 > 160
    reg [7:0] mem_adr_next;

    reg [1:0] byte_adr; //2'b11 is dev_id
    reg [1:0] byte_adr_next;

    reg [3:0] bit_adr; //0-7, 8 is ack (9 to stop?)
    reg [3:0] bit_adr_next;

    reg sda_next;
    reg scl_next;
    reg scl_freq_prev;
    reg scl_freq_prev_prev;

    reg txing;
    reg txing_next;

    reg stopping;
    reg stopping_next;

    reg active_next;

    always @(posedge clock) begin
        sda <= sda_next;
        scl <= scl_next;
        mem_adr <= mem_adr_next;
        byte_adr <= byte_adr_next;
        bit_adr <= bit_adr_next;
        scl_freq_prev <= scl_freq;
        scl_freq_prev_prev <= scl_freq_prev;
        txing <= txing_next;
        stopping <= stopping_next;
        active <= active_next;
    end

    always @* begin
        sda_next = sda;
        scl_next = scl;
        mem_adr_next = mem_adr;
        byte_adr_next = byte_adr;
        bit_adr_next = bit_adr;
        txing_next = txing;
        stopping_next = stopping;
        active_next = active;

        if (reset) begin
            sda_next = 1'b1;
            scl_next = 1'b1;
            mem_adr_next = 8'b0;
            byte_adr_next = 2'b11;
            bit_adr_next = 4'h7;
            txing_next = 1'b0;
            stopping_next = 1'b0;
            active_next = 1'b1;
        end else if (mem_adr >= 160) begin //done with transmissions
            sda_next = 1'b1;
            active_next = 1'b0;
        end else if (scl_freq && !scl_freq_prev) begin
            scl_next = 1'b1;
        end else if (scl_freq_prev && !scl_freq_prev_prev) begin //posedge scl - start, stop
            if (stopping) begin
                sda_next = 1'b1; //sda is ack:high, stopping:low, stop condition:high
                stopping_next = 1'b0;
                mem_adr_next = mem_adr + 8'b1;
            end else begin
                if (byte_adr == 2'b11 && bit_adr == 4'h7 && !txing) begin //start
                    txing_next = 1'b1;
                    sda_next = 1'b0;
                    //mem_adr_next = mem_adr + 8'b1;
                end else if (byte_adr == 2'b11 && bit_adr == 4'h7 && txing) begin //stop
                    txing_next = 1'b0;
                    stopping_next = 1'b1;
                end
            end

        end else if (!scl_freq && scl_freq_prev) begin // && txing) begin
            scl_next = 1'b0;
        end else if (!scl_freq_prev && scl_freq_prev_prev) begin //negedge scl - transmit
            if (stopping)
                sda_next = 1'b0;
            else if (txing) begin
                bit_adr_next = bit_adr - 4'b1;

                if (!bit_adr) begin
                    byte_adr_next = byte_adr + 2'b01;
                    bit_adr_next = 4'h8;
                end

                if (bit_adr == 4'h8)
                    sda_next = 1'b1;
                else if (byte_adr == 2'b11)
                    sda_next = !(!({`DEV_ID,1'b0} & (8'b1 << bit_adr)));
                else
                    sda_next = mem[mem_adr][{2'b10 - byte_adr, bit_adr[2:0]}];

            end
        end
    end
endmodule


module uart_jpeg_top(input                  osc_12m,

                     // hm01b0 connections
                     inout                  hm01b0_sda,
                     inout                  hm01b0_scl,

                     input [7:0]            hm01b0_pixdata,
                     input                  hm01b0_pixclk,
                     input                  hm01b0_hsync,
                     input                  hm01b0_vsync,

                     output                 hm01b0_mck,

                     // recieve debug LEDs from microcontroller
                     input  mcu_debug_signal,

                     // transmit to microcontroller
                     output uart_tx,
                     output vsync_out,
                     output spi_cs,

                     // using these as debug copies of sda and scl
                     output [3:0] leds);

    ///////////////////////////////////////////////
    // Debug GPIOs
    assign config_ps = 1'b1;

    //assign vsync_out = 1'bz;

    // assign hm01b0_trig_debug_led = uart_tx_config_copi;

    ////////////////////////////////////////////////
    // reset circuit
    wire power_on_reset;
    wire i2c_initializer_active;
    wire post_camera_init_reset = (power_on_reset || i2c_initializer_active);
    resetter r(.clock(osc_12m), .reset(power_on_reset));
    defparam r.count_maxval = 12000;

    ////////////////////////////////////////////////
    // i2c
    wire i2c_driver_clock;
    defparam i2c_div.N = 20;
    divide_by_n i2c_div(.clk(osc_12m), .reset(power_on_reset), .out(i2c_driver_clock));

    wire i2c_reset;
    resetter i2c_resetter(.clock(i2c_driver_clock), .reset(i2c_reset));
    i2c_initializer initializer(.clock(i2c_driver_clock),
                                .reset(i2c_reset),
                                .hm01b0_sda(hm01b0_sda),
                                .hm01b0_scl(hm01b0_scl),
                                .active(i2c_initializer_active));

    ////////////////////////////////////////////////
    // hm01b0 mck
    assign hm01b0_mck = osc_12m;

    ////////////////////////////////////////////////
    // trim edges of image
    wire trimmed_pixclk, trimmed_hsync, trimmed_vsync;
    vsync_hsync_roi trimmer(.pixclk_in(hm01b0_pixclk), .hsync_in(hm01b0_hsync), .vsync_in(hm01b0_vsync),
                            .pixclk_out(trimmed_pixclk), .hsync_out(trimmed_hsync), .vsync_out(trimmed_vsync));

    ////////////////////////////////////////////////
    // compress and send over UART
    wire jfpjc_vsync, jfpjc_hsync;
    wire [7:0] jfpjc_data_out;
    jfpjc jfpjc(.nreset(!post_camera_init_reset), .clock(osc_12m),
                .hm01b0_pixclk(trimmed_pixclk), .hm01b0_pixdata(hm01b0_pixdata),
                .hm01b0_hsync(trimmed_hsync), .hm01b0_vsync(trimmed_vsync),
                .hsync(jfpjc_hsync), .vsync(jfpjc_vsync), .data_out(jfpjc_data_out));
    defparam jfpjc.quant_table_file = "./quantization_table_jpeg_annex.hex";

    wire fstuff_vsync_out;
    wire fstuff_state;
    wire fstuff_data_out_valid;
    wire [7:0] fstuff_data_out;
    wire state;
    frame_end_stuffer fstuff(.clock(osc_12m), .reset(post_camera_init_reset),
                             .data_in_valid(jfpjc_hsync), .vsync_in(jfpjc_vsync), .data_in(jfpjc_data_out),
                             .data_out_valid(fstuff_data_out_valid), .data_out(fstuff_data_out),
                             .vsync_out(fstuff_vsync_out), .state(state));
    defparam fstuff.delimiter_length = 2 * 8;
    defparam fstuff.delimiter = 16'hffd9;

    spram_uart_dual_buffer outbuf(.clock(osc_12m), .reset(post_camera_init_reset),

                                  .data_in(fstuff_data_out),
                                  .data_in_valid(fstuff_data_out_valid),
                                  .vsync_in(fstuff_vsync_out),

                                  .uart_tx(uart_tx),
                                  .vsync_out(vsync_out));
    defparam outbuf.clock_divider = 7'd4;

    ////////////////////////////////////////////////////////////////
    // instantiate PWMs to drive LEDs from microcontroller inputs
    //     PD3 -> iCE40 pin 13 -> mcu_debug_signals[0] -> leds[0]
`ifdef NOT_DEFINED
    genvar leds_gi;
    generate
        for (leds_gi = 0; leds_gi < 1; leds_gi = leds_gi + 1) begin: generate_leds
            wire pwm_out;
            simple_constant_pwm pwm(clock, reset, pwm_out);
            //assign leds[leds_gi] = pwm_out & mcu_debug_signals[leds_gi];
            assign leds[leds_gi] = mcu_debug_signals[leds_gi];
        end
    endgenerate
`endif

    //wire led_pwm;
    //simple_constant_pwm pwm(clock, reset, led_pwm);

    // divide down vsync signal so we can visually see it.
    reg vsync_div;
    reg hm01b0_vsync_prev [0:2];
    always @(posedge osc_12m) begin
        if (reset) begin
            hm01b0_vsync_prev[0] <= 1'b0;
            hm01b0_vsync_prev[1] <= 1'b0;
            hm01b0_vsync_prev[2] <= 1'b0;
            vsync_div <= 1'b1;
        end else begin
            hm01b0_vsync_prev[0] <= hm01b0_vsync;
            hm01b0_vsync_prev[1] <= hm01b0_vsync_prev[0];
            hm01b0_vsync_prev[2] <= hm01b0_vsync_prev[1];

            if (hm01b0_vsync_prev[2] && !hm01b0_vsync_prev[1]) begin
                vsync_div <= ~vsync_div;
            end else begin
                vsync_div <= vsync_div;
            end
        end
    end


    //assign leds[3:0] = {hm01b0_vsync, 2'b11, ~(mcu_debug_signal)};
    assign leds[3:0] = {3'b111, ~(mcu_debug_signal)};
    //assign leds[3:0] = {vsync_out, post_camera_init_reset, hm01b0_vsync, hm01b0_hsync};
endmodule
