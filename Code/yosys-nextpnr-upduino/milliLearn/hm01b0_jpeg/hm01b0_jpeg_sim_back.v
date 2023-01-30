`timescale 1ns/100ps

`define DEV_ID 8'h24

module i2c_initializer(
    input clock, //should be 12mhz
    input reset,

    inout hm01b0_sda,
    inout hm01b0_scl,
    output mySda,
    output myScl
);
    reg sda,scl;
    wire in1;
    wire in2;
  
    assign hm01b0_sda = sda ? 1'b1 : 1'b0;
    assign hm01b0_scl = scl ? 1'b1 : 1'b0;

    wire scl_freq;
    //A11 = 12000000/(440*(2^7)) = 213.068
    defparam div.N = 213; //120 for 100khz, 30 for 400khz
    divide_by_n div(
        .clk(clock),
        .reset(reset),
        .out(scl_freq)
    );

    assign mySda = sda;
    assign myScl = scl;
    reg [23:0] mem [0:159]; // 0 to 158
    //integer i;
    initial begin
        $readmemh("new_hex.hex", mem);
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
    reg stopping_2;

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
    end

    always @* begin
        sda_next = sda;
        scl_next = scl;
        mem_adr_next = mem_adr;
        byte_adr_next = byte_adr;
        bit_adr_next = bit_adr;
        txing_next = txing;
        stopping_next = stopping;

        if (reset) begin
            sda_next = 1'b1;
            scl_next = 1'b1;
            mem_adr_next = 8'b0;
            byte_adr_next = 2'b11;
            bit_adr_next = 4'h7;
            txing_next = 1'b0;
            stopping_next = 1'b0;
            stopping_2 = 1'b0;
        end else if (mem_adr >= 160) begin //done with transmissions
            sda_next = 1'b1;
        end else if (scl_freq && !scl_freq_prev) begin
            scl_next = 1'b1;
        end else if (scl_freq_prev && !scl_freq_prev_prev) begin //posedge scl - start, stop
              if(!stopping && stopping_2)
                begin
                mem_adr_next = mem_adr + 8'b1;
                end


            if (stopping_2) begin
                sda_next = 1'b1; //sda is ack:high, stopping:low, stop condition:high
                stopping_next = 1'b0;
                stopping_2 = 1'b0;
                // scl_next = 1'b0;
                
            end


          
            
            if(stopping)
                begin
                    stopping_2 = 1'b1;
                    
            end

          
            
             

            
            else begin
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
            scl_next = (stopping_2 && !sda) ? 1'b1 : 1'b0;

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


module hm01b0_jpeg_top(input                  osc_12m,

                       // hm01b0 connections
                       inout                  hm01b0_sda,
                       inout                  hm01b0_scl,

                       input [7:0]            hm01b0_pixdata,
                       input                  hm01b0_pixclk,
                       input                  hm01b0_hsync,
                       input                  hm01b0_vsync,

                       output                 hm01b0_mck,

                       // assorted debug connections
                       output config_ps,
                       output uart_tx_config_copi,
                       output hm01b0_trig_debug_led,

                       // using these as debug copies of sda and scl
                       output [6:0] gpio,

                       output gpio43_hiz);
    ///////////////////////////////////////////////
    // Debug GPIOs
    assign config_ps = 1'b1;

    assign hm01b0_trig_debug_led = uart_tx_config_copi;
    assign gpio43_hiz = (hm01b0_vsync) ? (1'bz) : (1'b0);

    ////////////////////////////////////////////////
    // reset circuit
    wire reset;
    resetter r(.clock(osc_12m), .reset(reset));
    defparam r.count_maxval = 5;

    ////////////////////////////////////////////////
    // i2c
    wire mySda;
    wire myScl;
    
    i2c_initializer initializer(.clock(osc_12m), .reset(reset), .hm01b0_sda(hm01b0_sda), .hm01b0_scl(hm01b0_scl), .mySda(mySda), .myScl(myScl));

    ////////////////////////////////////////////////
    // hm01b0 mck
    assign hm01b0_mck = osc_12m;

   
    assign gpio[1:0] = {mySda, myScl};
endmodule
