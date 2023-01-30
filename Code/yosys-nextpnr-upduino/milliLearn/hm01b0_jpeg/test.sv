module test;


   reg osc_12m;

   wire hm01b0_sda;
   
   wire hm01b0_scl;
   
   hm01b0_jpeg_top dut(.osc_12m(osc_12m),           
                       .hm01b0_sda(hm01b0_sda), 
                       .hm01b0_scl(hm01b0_scl)                                   
		       );
always #10 osc_12m = ~osc_12m;

integer idx; 
   initial 
   begin
      osc_12m = 1'b0;
      $dumpfile("test.vcd");
      $dumpvars(0,dut); 
      // $dumpvars(0,dut.myScl); 
      
      for (idx = 0; idx < 160; idx = idx + 1) $dumpvars(0, dut.initializer.mem[idx]);

      #10000000 $finish;
   end
   

endmodule
