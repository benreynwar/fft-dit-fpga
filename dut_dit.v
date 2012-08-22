// Copyright (c) 2012 Ben Reynwar
// Released under MIT License (see LICENSE.txt)

// This is simply a wrapper around the dit module so that it can be accessed from the
// myhdl test bench.

module dut_dit;
   reg                          clk;
   reg                          rst_n;
   reg [`X_WDTH*2-1:0]          din;
   wire [`X_WDTH*2-1:0]         dout;
   reg                          din_nd;
   wire                         dout_nd;
   wire                         overflow;
   
   initial begin
	  $from_myhdl(clk, rst_n, din, din_nd);
	  $to_myhdl(dout, dout_nd, overflow);
   end
   
   dit #(`N, `NLOG2, `TF_WDTH, `X_WDTH) dut (clk, rst_n, din, din_nd, dout, dout_nd, overflow);
   
endmodule  