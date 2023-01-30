`timescale 1ns/100ps

module uart_lfsr_tb();
    reg clock;
    wire spi_cs;
    wire [3:0] leds;
    wire tx;

    uart_lfsr_top top(
        .osc_12m(clock),
        .spi_cs(spi_cs),
        .leds(leds),
        .tx(tx)
    );

    localparam SYS_CLOCK_FREQ = 12000000.0;
    always begin
        clock = 1'b0;
        #((1E9 / SYS_CLOCK_FREQ) / 2);
        clock = 1'b1;
        #((1E9 / SYS_CLOCK_FREQ) / 2);
    end

    integer i;
    initial begin
        $dumpfile("uart_lfsr.vcd");
        $dumpvars;

        for (i = 0; i < 120000; i = i + 1) begin : loop
            @(posedge clock);
        end

        $finish;
    end

endmodule




