`timescale 1ns/100ps

module spram_uart_dual_buffer_tb();
    reg clock;
    reg reset;

    reg [7:0] data_in;
    reg       data_in_valid;
    reg       vsync_in;

    wire uart_tx;
    wire vsync_out;

    spram_uart_dual_buffer buffer(.clock(clock),
                                  .reset(reset),

                                  .data_in(data_in),
                                  .data_in_valid(data_in_valid),
                                  .vsync_in(vsync_in),

                                  .uart_tx(uart_tx),
                                  .vsync_out(vsync_out));

    // generate 10MHz system clock
    integer cycle_count;
    always begin
        clock = 1'b0;
        #50;
        clock = 1'b1;
        cycle_count = cycle_count + 1;
        #50;
    end


    localparam clocks_per_frame = 100000;

    reg [7:0] data_in_next = 'h00;
    integer frame_num;
    integer i, j;
    integer interbyte_delay;
    integer interframe_delay;
    integer bytes_this_frame;
    localparam frames_to_send = 10;
    initial begin
        $dumpfile("uart_vsync_buffer_tb.vcd");
        $dumpvars(0, spram_uart_dual_buffer_tb);

        // dump array variables
        for (i = 0; i < 2; i = i + 1) begin
            $dumpvars(1, buffer.vsync_in_prev[i]);
            $dumpvars(1, buffer.bytes_in_buffer[i]);
            $dumpvars(1, buffer.spram_address[i]);
            $dumpvars(1, buffer.spram_wren[i]);
            $dumpvars(1, buffer.spram_data_out[i]);
            $dumpvars(1, buffer.spram_data_in[i]);
        end


        cycle_count = 0;

        // hold in reset for 2 clock cycles
        reset = 1'b1;
        @(posedge clock); @(posedge clock); #1;
        reset = 1'b0;

        // Send frames
        for (frame_num = 0; frame_num < frames_to_send; frame_num = frame_num + 1) begin
            // bytes per frame is a random number in [100, 228)
            bytes_this_frame = ($urandom % 128) + 100;
            vsync_in = 1'b1;

            data_in = 'h00;

            for (i = 0; i < bytes_this_frame; i = i + 1) begin
                data_in_valid = 1'b0;
                data_in = 'hxx;

                // random inter-byte delay of [0, 256) clock cycles.
                interbyte_delay = ($urandom % 256);
                for (j = 0; j < interbyte_delay; j = j + 1) begin
                    @(posedge clock); #1;
                end

                // present data on input for one clock cycle
                data_in_valid = 1'b1;
                data_in = data_in_next;
                data_in_next = data_in_next + 'h1;
                @(posedge clock); #1;
            end

            data_in_next = 'h00;

            // Each frame should last exactly clocks_per_frame clock cycles. Insert an appropriate
            // amount of interframe delay.
            data_in_valid = 1'b0;
            data_in = 'hxx;
            vsync_in = 1'b0;

            while (cycle_count < ((frame_num + 1) * clocks_per_frame)) begin
                @(posedge clock); #1;
            end

            $display("Frame %d finished", frame_num);
        end

        $finish;
    end

endmodule
