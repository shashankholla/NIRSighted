/**
 * This module takes parallel data in and transmits it over UART.
 *
 * Because data presented at the input port might arrive much faster than you'll be able to
 * transmit it over uart, you'll need to maintain a buffer. To save other on-chip resources, this
 * buffer should be implemented using at least 2 single-port RAM (SPRAM) blocks. To accomodate the
 * use of SPRAM, it's ok for your module to wait an arbitrary amount of time before sending a
 * byte that it has recieved. If RAM #0 is being filled with data, and RAM #1 has no more data left
 * to send, you can wait until RAM #0 is completely full before you start transmitting its data.
 *
 * On average, data will not arrive faster than you can transmit it.
 */

`timescale 1ns/100ps


`ifndef YOSYS
module SB_SPRAM256KA (
    input [15:0] DATAIN,
    input [13:0] ADDRESS,
    input [3:0] MASKWREN,
    input WREN,
    input CHIPSELECT,//
    input CLOCK,
    input STANDBY,//
    input SLEEP,//
    input POWEROFF,//
    output reg [15:0] DATAOUT
);
    //ignore standby,sleep,poweroff,chipselect
    //could standby/sleep/poweroff the spram that finished sending bytes to uart?

    reg [15:0] data [(2**14) - 1:0];

    always @(posedge CLOCK) begin
        if (WREN) begin //write to spram
            if (MASKWREN == 4'b0011) begin
                data[ADDRESS][7:0] <= DATAIN[7:0];
            end else if (MASKWREN == 4'b1100) begin
                data[ADDRESS][15:8] <= DATAIN[15:8];
            end else begin
                //should never happen
            end
        end else begin //read from spram
            DATAOUT <= data[ADDRESS];
        end
    end
endmodule
`endif

`define WRITE_0 (1'b0) //input -> spram_0
`define WRITE_1 (1'b1) //input -> spram_1
`define TX_0 (1'b1) //spram_0 -> ut
`define TX_1 (1'b0) //spram_1 -> ut

module spram_uart_buffer(
    input       clock,
    input       reset,
    input [7:0] data_in,
    input       data_in_valid,
    output  reg uart_tx
);
    parameter max_address = 15'h7FFF; //7FFF for full 32kB
    parameter clock_divider = 7'd104; //13 or 104 for standard bauds from 12mhz:
    //12_000_000 / 13  = 923_076.923 ~ 921_600
    //12_000_000 / 104 = 115_384.615 ~ 115_200

    wire baud_clock;
    defparam div.N = clock_divider;
    divide_by_n div(
        .clk(clock),
        .reset(reset),
        .out(baud_clock)
    );

    reg [7:0] data_in_reg;
    reg [7:0] data_in_reg_next; //put data_in here to be spram input next cycle

    reg [14:0] address_0; //lsb is byte select
    reg [14:0] address_0_next;

    reg [14:0] address_1;
    reg [14:0] address_1_next;

    reg wren_0; //write enable when wren == 1
    reg wren_0_next;

    reg wren_1;
    reg wren_1_next;

    wire [15:0] data_out_0; //driven by spram
    wire [15:0] data_out_1;

    SB_SPRAM256KA spram_0(
        .DATAIN({data_in_reg, data_in_reg}),
        .ADDRESS(address_0[14:1]),
        .MASKWREN({{2{address_0[0]}}, {2{!address_0[0]}}}), //0011 or 1100 for lsB or msB
        .WREN(wren_0),
        .CHIPSELECT(1'b1),
        .CLOCK(clock),
        .STANDBY(1'b0),
        .SLEEP(1'b0),
        .POWEROFF(1'b1),
        .DATAOUT(data_out_0)
    );

    SB_SPRAM256KA spram_1(
        .DATAIN({data_in_reg, data_in_reg}),
        .ADDRESS(address_1[14:1]),
        .MASKWREN({{2{address_1[0]}}, {2{!address_1[0]}}}),
        .WREN(wren_1),
        .CHIPSELECT(1'b1),
        .CLOCK(clock),
        .STANDBY(1'b0),
        .SLEEP(1'b0),
        .POWEROFF(1'b1),
        .DATAOUT(data_out_1)
    );

    reg uart_data_valid;
    reg uart_data_valid_next;

    reg [7:0] uart_data;
    reg [7:0] uart_data_next;

    wire uart_tx_next;

    wire uart_busy; //driven by uart_tx

    uart_tx ut(.clock(clock),
               .reset(reset),
               .baud_clock(baud_clock),
               .data_valid(uart_data_valid),
               .data(uart_data),
               .uart_tx(uart_tx_next),
               .uart_busy(uart_busy)
    );

    reg state;
    reg state_next;

    reg done_txing; //last transmit by spram -> uart state machine
    reg done_txing_next;

    reg tx_last_clock; //uart_busy takes a cycle to update, so this fills in for that cycle
    reg tx_last_clock_next;

    reg state_change; //this is 1 on the posedge of state changing
    reg state_change_next;

    reg reset_flag; //set to denote spram being undefined due to reset
    reg reset_flag_next;

    always @* begin
        data_in_reg_next = data_in;
        address_0_next = address_0;
        address_1_next = address_1;
        if (reset) begin
            state_next = `WRITE_1; //== `TX_0

            address_0_next = max_address; //immediate state change
            address_1_next = max_address;
            wren_0_next = 1'b0;
            wren_1_next = 1'b0;

            uart_data_valid_next = 1'b0;
            uart_data_next = 8'hxx;

            done_txing_next = 1'b1;
            tx_last_clock_next = 1'b0;
            state_change_next = 1'b0;
            reset_flag_next = 1'b1;
        end else begin
            //input -> spram
            case (state)
                `WRITE_0: begin
                    //write
                    if (data_in_valid) begin
                        if (address_0 == max_address) begin
                            state_next = `WRITE_1;
                            address_0_next = 15'b0;
                            wren_0_next = 1'b0;
                            wren_1_next = 1'b1;
                            state_change_next = 1'b1;
                        end else begin
                            state_next = `WRITE_0;
                            address_0_next = address_0 + 15'b1;
                            wren_0_next = 1'b1;
                            wren_1_next = 1'b0;
                            state_change_next = 1'b0;
                        end
                    //no write
                    end else begin
                        state_next = `WRITE_0;
                        address_0_next = address_0;
                        wren_0_next = 1'b0;
                        wren_1_next = 1'b0;
                        state_change_next = 1'b0;
                    end
                end

                `WRITE_1: begin
                    if (data_in_valid) begin
                        if (address_1 == max_address) begin
                            state_next = `WRITE_0;
                            address_1_next = 15'b0;
                            wren_1_next = 1'b0;
                            wren_0_next = 1'b1;
                            state_change_next = 1'b1;
                        end else begin
                            state_next = `WRITE_1;
                            address_1_next = address_1 + 15'b1;
                            wren_1_next = 1'b1;
                            wren_0_next = 1'b0;
                            state_change_next = 1'b0;
                        end
                    end else begin
                        state_next = `WRITE_1;
                        address_1_next = address_1;
                        wren_1_next = 1'b0;
                        wren_0_next = 1'b0;
                        state_change_next = 1'b0;
                    end
                end
            endcase

            //spram -> uart
            case (state)
                `TX_0: begin
                    //happens right before and after state changes. extra cycle for correct data
                    if (state_change || state_change_next) begin
                        uart_data_valid_next = 1'b0;
                        uart_data_next = 8'hxx;
                        address_0_next = 1'b0;
                        tx_last_clock_next = 1'b0;
                        done_txing_next = 1'b0;
                    //no transmit
                    end else if (uart_busy || done_txing || tx_last_clock || reset_flag) begin
                        uart_data_valid_next = 1'b0;
                        uart_data_next = 8'hxx;
                        address_0_next = address_0;
                        tx_last_clock_next = 1'b0;
                        done_txing_next = done_txing;
                    //transmit
                    end else begin
                        uart_data_valid_next = 1'b1;
                        uart_data_next = address_0[0] ? data_out_0[15:8] : data_out_0[7:0];
                        address_0_next = address_0 + 15'b1;
                        tx_last_clock_next = 1'b1;
                        done_txing_next = (address_0 == max_address) ? 1'b1 : 1'b0;
                    end
                end

                `TX_1: begin
                    if (state_change || state_change_next) begin
                        uart_data_valid_next = 1'b0;
                        uart_data_next = 8'hxx;
                        address_1_next = 1'b0;
                        tx_last_clock_next = 1'b0;
                        done_txing_next = 1'b0;
                    end else if (uart_busy || done_txing || tx_last_clock || reset_flag) begin
                        uart_data_valid_next = 1'b0;
                        uart_data_next = 8'hxx;
                        address_1_next = address_1;
                        tx_last_clock_next = 1'b0;
                        done_txing_next = done_txing;
                    end else begin
                        uart_data_valid_next = 1'b1;
                        uart_data_next = address_1[0] ? data_out_1[15:8] : data_out_1[7:0];
                        address_1_next = address_1 + 15'b1;
                        tx_last_clock_next = 1'b1;
                        done_txing_next = (address_1 == max_address) ? 1'b1 : 1'b0;
                    end
                end
            endcase
            reset_flag_next = (state_change && state == `WRITE_1) ? 1'b0 : reset_flag; //turn off at second state change
        end
    end

    always @(posedge clock) begin
        state <= state_next;

        data_in_reg <= data_in_reg_next;
        address_0 <= address_0_next;
        address_1 <= address_1_next;
        wren_0 <= wren_0_next;
        wren_1 <= wren_1_next;

        uart_data_valid <= uart_data_valid_next;
        uart_data <= uart_data_next;
        uart_tx <= uart_tx_next;

        done_txing <= done_txing_next;
        tx_last_clock <= tx_last_clock_next;
        state_change <= state_change_next;
        reset_flag <= reset_flag_next;
    end
endmodule
