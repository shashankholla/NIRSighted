`timescale 1ns/100ps

module blinker(input       clock,
               input       reset,
               output [3:0]  blink);
    reg [23:0] counter;

    always @* begin
        case (counter[22:21])
            2'b00: blink = 4'b1110;
            2'b01: blink = 4'b1101;
            2'b10: blink = 4'b1011;
            2'b11: blink = 4'b0111;
        endcase // case (counter[23:22])
    end

    always @(posedge clock) begin
        if (!reset) begin
            counter <= counter + 24'h1;
        end else begin
            counter <= 24'h0;
        end
    end
endmodule

// The name blinky_top is arbitrary. It just needs to be specified in the yosys synthesis script
// with the -top argument passed into synth_ice40
module blinky_top(input osc_12m,
                  output spi_cs,
                  output [3:0] leds);

    assign spi_cs = 1'b1;

    // This oscillator is a hard IP core inside the ice40.
    /*wire clk_48;
    SB_HFOSC u_hfosc(.CLKHFPU(1'b1),
		     .CLKHFEN(1'b1),
		     .CLKHF(clk_48));*/

    // Module "resetter" is defined in ../common/util.v
    wire reset;
    resetter r(.clock(osc_12m),
               .reset(reset));

    blinker b(.clock(osc_12m),
              .reset(reset),
              .blink(leds));
endmodule
