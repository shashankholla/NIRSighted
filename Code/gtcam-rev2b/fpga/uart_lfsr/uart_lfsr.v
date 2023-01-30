`timescale 1ns/100ps

module uart_tx(
    input       clock,
    input       reset,
    input       baud_clock,
    input       data_valid,
    input [7:0] data,
    output reg  uart_tx,
    output reg  uart_busy
);
    reg [9:0] shift_register; //1+8+1 for start+byte+stop
    reg [3:0] shift_counter;
    reg baud_clock_prev;
    always @(posedge clock) begin
        if (reset) begin
            shift_register <= 10'bxxx;
            shift_counter <= 4'b0;
            uart_tx <= 1'b1;
            uart_busy <= 1'b0;
        end else begin
            if (uart_busy) begin //sending byte
                if (shift_counter < 4'd10) begin //still sending
                    if (baud_clock && !baud_clock_prev) begin //posedge baud_clock
                        shift_register <= shift_register >> 1'b1;
                        shift_counter <= shift_counter + 4'h1;
                        uart_busy <= 1'b1;
                        uart_tx <= shift_register[0];
                    end else begin //waiting for baud
                        shift_register <= shift_register;
                        shift_counter <= shift_counter;
                        uart_busy <= uart_busy;
                        uart_tx <= uart_tx;
                    end
                end else begin //done sending
                    shift_register <= 10'hxxx;
                    shift_counter <= 4'b0;
                    uart_busy <= 1'b0;
                    uart_tx <= 1'b1; //this is always going to be 1 (stop bit) right?
                end
            end else begin //waiting to get byte
                uart_tx <= 1'b1;
                if (data_valid) begin
                    shift_register <= {1'b1, data, 1'b0}; //{stop,data,start}
                    shift_counter <= 4'h0;
                    uart_busy <= 1'b1;
                end else begin
                    shift_register <= shift_register;
                    shift_counter <= shift_counter;
                    uart_busy <= uart_busy;
                end
            end
        end
        baud_clock_prev <= baud_clock;
    end
endmodule


module uart_lfsr_top(
    input osc_12m,
    output spi_cs,
    output [3:0] leds,
    output tx,
    output reg vsync
);
    assign leds = 4'hF;
    assign spi_cs = 1'b1;

    wire reset;
    resetter #(
               .count_maxval(10000)
    ) r (
         .clock(osc_12m),
         .reset(reset));

    //parameter clock_divider = 7'd104; //13 or 104 for standard bauds from 12mhz:
    //12_000_000 / 13  = 923_076.923 ~ 921_600
    //12_000_000 / 104 = 115_384.615 ~ 115_200
    //12_000_000 / 1250 = 9600

    //localparam CLOCK_DIVIDER = 1250;
    //localparam CLOCK_DIVIDER = 104;
    localparam CLOCK_DIVIDER = 6;

    wire baud_clock;
    divide_by_n #(
        .N(CLOCK_DIVIDER)
    ) div0 (
        .clk(osc_12m),
        .reset(reset),
        .out(baud_clock)
    );

    reg data_valid;
    reg [7:0] data;
    wire uart_busy;
    uart_tx uart(
        .clock(osc_12m),
        .reset(reset),
        .baud_clock(baud_clock),
        .data_valid(data_valid),
        .data(data),
        .uart_tx(tx),
        .uart_busy(uart_busy)
    );

    wire new_byte_clock; //changes when new byte is sent over uart
    divide_by_n #(
        .N(CLOCK_DIVIDER*40) //give some space for now. later, just check uart_busy
    ) div1 (
        .clk(osc_12m),
        .reset(reset),
        .out(new_byte_clock)
    );

    //reg new_byte_clock_last;
    //assign data_valid = new_byte_clock ^ new_byte_clock_last;

    /**
     * hacky logic block to advance uart data
     */
    reg [7:0] uart_framestart_value;
    reg uart_busy_prev [0:1];
    reg vsync_prev [0:1];
    always @(posedge osc_12m) begin
        if (reset) begin
            uart_busy_prev[0] <= 1'b0;
            uart_busy_prev[1] <= 1'b0;
            vsync_prev[0] <= 1'b0;
            vsync_prev[1] <= 1'b0;
            data <= 8'h00;
            uart_framestart_value <= 8'h01;
        end else begin
            uart_busy_prev[1] <= uart_busy_prev[0];
            uart_busy_prev[0] <= uart_busy;

            vsync_prev[1] <= vsync_prev[0];
            vsync_prev[0] <= vsync;

            if (uart_busy_prev[0] && !uart_busy_prev[1]) begin
                // on rising edge of uart_busy, increment 'data'.
                data <= data + 8'h01;
                uart_framestart_value <= uart_framestart_value;
            end else if (vsync_prev[0] && !vsync_prev[1]) begin
                // on rising edge of vsync, reset 'data'.
                data <= uart_framestart_value;
                uart_framestart_value <= uart_framestart_value + 8'h01;
            end else begin
                data <= data;
                uart_framestart_value <= uart_framestart_value;
            end
        end
    end

    /**
     * hacky logic to simulate vsync:
     *    * once every 16 baud_clock, send a new character
     *    * when baud_clock_count in [0x01_0000, 0x01_8000), hold vsync high
     *    * when baud_clock_count in
     */
    reg [23:0] baud_clock_count;
    always @(posedge baud_clock) begin
        if (reset) begin
            baud_clock_count <= 24'h00_0000;
            data_valid <= 1'b0;
            vsync <= 1'b1;
        end else begin
            if (baud_clock_count < 24'd60272) begin
                // baud_clock_count in range [0x00_0000, 0x04_0000)
                data_valid <= (baud_clock_count[3:0] == 4'h0) ? 1'b1 : 1'b0;
                baud_clock_count <= baud_clock_count + 24'h00_0001;
                vsync <= 1'b0;
            end else if (baud_clock_count < 24'h01_8000) begin
                // baud_clock_count in range [0x04_0000, 0x04_8000)
                data_valid <= 1'b0;
                baud_clock_count <= baud_clock_count + 24'h00_0001;
                vsync <= 1'b1;
            end else if (baud_clock_count < 24'h02_0000) begin
                data_valid <= 1'b0;
                baud_clock_count <= baud_clock_count + 24'h00_0001;
                vsync <= 1'b0;
            end else begin
                data_valid <= 1'b0;
                baud_clock_count <= 24'h00_0000;
                vsync <= 1'b0;
            end
        end
    end

`ifdef NOT_DEFINED
    lfsr l(
        .clock(osc_12m),
        .reset(reset),
        .data_valid(data_valid),
        .out(data)
    );
`endif
endmodule
