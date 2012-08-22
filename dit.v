// Copyright (c) 2012 Ben Reynwar
// Released under MIT License (see LICENSE.txt)

// FFT - Decimation in Time
// The produced FFT is scaled down by a factor of N to prevent overflow.

// The Butterfly module that we are using assume that TF_WDTH is the same
// as X_WDTH.
// TF_WDTH must be the same as X_WDTH

module dit
   #(
     // Length of FFT vector.
     parameter N = 16,
     // Base two log of N
     parameter NLOG2 = 4,
     // Number of bits in vector values (double this value for a complex number).
     parameter X_WDTH = 8,
      // Number of bits in twiddle factor values. (must be equal to X_WDTH at the moment)
     parameter TF_WDTH = 8,
     // Whether to run in debug mode.
     parameter DEBUGMODE = 0
     )
   (
    // The clock signal.
    input wire                clk,
    // Normally set to 1.  Set to 0 to reset module.
    input wire                rst_n,
    // Input value.
    // Within each complex number the real part is at the low end and the complex
    // at the high end.
    input wire [2*X_WDTH-1:0] in_x,
    // Set to 1 when new data placed in in_x.
    input wire                in_nd,
    // Output value.
    output reg [2*X_WDTH-1:0] out_x,
    // Set to 1 when new data is placed in out_x.
    output reg                out_nd,
    // Set to 1 when can't keep up with input data.
    output reg                overflow
    );

   `define MSG_DEBUG(g) if(DEBUGMODE) $display("DEBUG : %m:", g)
   `define MSG_ERROR(g) $display("ERROR : %m:", g)
   
   /******************************/
   /* Define global data buffers */
   /******************************/

   // Input buffer.
   reg [X_WDTH*2-1:0]           bufferin0[N-1:0];
   reg                          bufferin_full0_A;
   reg                          bufferin_full0_B;
   wire                         bufferin_full0;
   assign bufferin_full0 = bufferin_full0_A + bufferin_full0_B;
   reg [X_WDTH*2-1:0]           bufferin1[N-1:0];
   reg                          bufferin_full1_A;
   reg                          bufferin_full1_B;
   wire                         bufferin_full1;
   assign bufferin_full1 = bufferin_full1_A + bufferin_full1_B;
   reg                          bufferin_write_switch;
   reg                          bufferin_read_switch;
   wire                         bufferin_read_full;
   wire                         bufferin_write_full;
   assign bufferin_read_full = bufferin_read_switch?bufferin_full1:bufferin_full0;
   assign bufferin_write_full = bufferin_write_switch?bufferin_full1:bufferin_full0;
   // Working buffers.
   reg [X_WDTH*2-1:0]           bufferX[N-1:0];
   reg [X_WDTH*2-1:0]           bufferY[N-1:0];
   // Output buffer.
   reg [X_WDTH*2-1:0]           bufferout[N-1:0];
   // Whether the output buffer is full.
   // We have two registers since they are drive by different processes.
   // 'A' flips back and forth as the buffer is fulled.
   // 'B' flips back and forth as the buffer is emptied.
   reg                          bufferout_full_A;
   reg                          bufferout_full_B;
   wire                         bufferout_full;
   assign bufferout_full = bufferout_full_A + bufferout_full_B;
   // Whether the buffer contains good data. (i.e. not old)
   // Data should not be read from a buffer unless the correponding
   // updated value is 1.
   reg [N-1:0]                  updatedX;
   reg [N-1:0]                  updatedY;

   /*******************************************************/
   /*                                                     */
   /* Define logic for receiving samples and placing in   */
   /* an input buffer.                                    */
   /*                                                     */
   /*******************************************************/

   reg [NLOG2-1:0]              bufferin_addr;
   
   initial
     begin
        bufferin_addr <= {NLOG2{1'b0}};
        bufferin_full0_A <= 1'b0;
        bufferin_full1_A <= 1'b0;
        bufferin_write_switch <= 1'b0;
        overflow <= 1'b0;
     end

   always @ (posedge clk or negedge rst_n)
     begin
        if (!rst_n)
          begin
             bufferin_addr <= {NLOG2{1'b0}};
             bufferin_full0_A <= 1'b0;
             bufferin_full1_A <= 1'b0;
             bufferin_write_switch <= 1'b0;
             overflow <= 1'b0;
          end
        else
          begin
             if (in_nd)
               begin
                  // Check for overflowed data.
                  if (bufferin_write_full)
                    overflow <= 1'b1;
                  if (bufferin_write_switch)
                    bufferin1[bufferin_addr] <= in_x;
                  else
                    bufferin0[bufferin_addr] <= in_x;
                  bufferin_addr <= bufferin_addr + 1;
                  if (&bufferin_addr)
                    begin
                       bufferin_write_switch <= ~bufferin_write_switch;
                       if (bufferin_write_switch)
                         bufferin_full1_A <= ~bufferin_full1_A;
                       else
                         bufferin_full0_A <= ~bufferin_full0_A;
                    end
               end
          end
     end
   
   /*******************************************************/
   /*                                                     */
   /* Define logic for emitting samples from the output   */
   /* buffer.                                             */
   /*                                                     */
   /*******************************************************/

   reg [NLOG2-1:0]              bufferout_addr;

   initial
     begin
        bufferout_addr <= {NLOG2{1'b0}};
        bufferout_full_B <= 1'b0;
        out_nd <= 1'b0;
     end
   
   always @ (posedge clk or negedge rst_n)
     begin
        if (!rst_n)
          begin
             bufferout_addr <= {NLOG2{1'b0}};
             bufferout_full_B <= 1'b0;
             out_nd <= 1'b0;
          end
        else
          begin
             if (bufferout_full)
               begin
                  out_x <= bufferout[bufferout_addr];
                  out_nd <= 1'b1;
                  bufferout_addr <= bufferout_addr + 1;
                  if (&bufferout_addr)
                    bufferout_full_B <= ~bufferout_full_B;
               end
             else
               out_nd <= 1'b0;
          end
     end
   
   /********************************************************/
   /*                                                      */
   /* Define FSM that passes data to the BF module.        */
   /*                                                      */
   /********************************************************/

   reg [1:0] fsm_state;
   /* Define the control FSM states. */
   localparam  [1:0]  FSM_ST_INIT = 0;
   localparam  [1:0]  FSM_ST_IDLE = 1;
   localparam  [1:0]  FSM_ST_CALC = 2;
   localparam  [1:0]  FSM_ST_SEND = 3;

   /*
    Calculation that determine which positions we should read from and write to
    for with the butterfly module.
    
    If we have a series x_n that we want to get the DFT of, X_k we can write X_k in
    terms of E_k and O_k where E_k and O_k are the DFTs of the even and odd components
    of x_n respectively.

    for k<N/2  : X_k = E_k + exp(-2*pi*i*k/N)*O_k
    for k>=N/2 : X_k = E_{k-N/2} - exp(-2*pi*{k-N/2}/N)*O_{k-N/2}
    We use this relationship to calculate the DFT of x_n in a series of stages.  AFter the
    final stage the output is X_k.  After the second to last stage the output is an
    interleaving of E_k and O_k.
    
    At some general stage we have S interleaved series.
    
    So if X_k is the j'th series in a stage and P_n is the n'th output in that stage:
    
    X_k = P_{k*S+j}
    E_k is from a stage with 2*S series and it is in the j'th series in the stage
    O_k is from a stage with 2*S series and it is in the (S+j)'th series in stage
    Let Q_n be the n'th output of the stage before P.
    E_k = Q_{k*2*S+j}
    O_k = Q_{k*2*S+S+j}

    Also let T_n = exp(-2*pi*i*n/M)
        
    M = N*S (total number of items in stage output)
    P_{k*S+j}     = Q_{2*k*S+j} + T_{k*S} * Q_{k*2*S+S+j}
    P_{k*S+j+M/2} = Q_{2*k*S+j} - T_{k*S} * Q_{k*2*S+S+j}
    
    We'll give these addresses names:
    out0_addr = k*S+j
    out1_addr = k*S+j+M/2
    in0_addr = 2*k*S+j
    in1_addr = 2*k*S+S+j
    
    Now we assume we know out0_addr and try to get efficient ways to calculate the
    other addresses.

    out0_addr = k*S+j   (j ranges from 0 to S-1, and S is a multiple of two)
    If we look at out0_addr in binary the lowest log2(S) bits give the value of j
    and the highest log2(N) bits give the value for k.
    */
    
   // Number of series in the stage we are writing to.
   reg [NLOG2-1:0] S;
   // Contains a 1 for the bits that give j from out0_addr (i.e. which series).
   reg [NLOG2-1:0] series_bits;			
   reg [NLOG2-1:0] out0_addr;
   // Functions of the above 3 registers.
   wire [NLOG2-1:0] in0_addr;
   wire [NLOG2-1:0] in1_addr;
   wire [NLOG2-1:0] out1_addr;
   wire [NLOG2-2:0] tf_addr;
   
   //To get in0_addr we leave the lowest log2(S) bits alone but we shift the log2(N)
   //highest bits to the left (high is to left).
   
   //To get in1_addr we add S to in0_addr.

   // out1_addr = out0+addr + M/2
   // We simply flip the highest bit from 0 to 1 which adds M/2.
   assign out1_addr = {1'b1, out0_addr[NLOG2-2:0]};
   // in0_addr = 2*k*S+j
   // (out0_addr & series_bits) = j
   // (out0_addr & ~series_bits) = k*S
   // Since the bits don't overlap we can add them with an OR.
   assign in0_addr = (out0_addr & series_bits) | ((out0_addr & ~series_bits)<<1);
   assign in1_addr = in0_addr + S;
   // (out0_addr & ~series_bits) = k*S
   assign tf_addr = out0_addr & ~series_bits;
   // Set to 1 when x_nd is set to 1 from the last BF calculation of the FFT.
   reg                          finished;
   // Which buffer we are reading from.
   // 1 if we are reading from X.
   // 0 if we are reading from Y.
   reg                          readbuf_switch;
   // We want readbuf_switch delayed by one step to send into the BF module.
   // The is because readbuf_switch may have changed since the values being
   // sent in were read.
   reg                          readbuf_switch_old;
   // Whether it is the first stage.
   wire                           first_stage;
   assign first_stage = (S == {1'b1,{NLOG2-1{1'b0}}});
   // Whether it is the last stage.
   wire                           last_stage;
   assign last_stage = (S == 1);
   // Inputs in to the BF module
   wire [2*X_WDTH-1:0]            in0;
   wire [2*X_WDTH-1:0]            in1;
   assign in0 = first_stage?(bufferin_read_switch?bufferin1[in0_addr]:bufferin0[in0_addr]):(readbuf_switch?bufferX[in0_addr]:bufferY[in0_addr]);
   assign in1 = first_stage?(bufferin_read_switch?bufferin1[in1_addr]:bufferin0[in1_addr]):(readbuf_switch?bufferX[in1_addr]:bufferY[in1_addr]);
   // Whether the two inputs have been updated.
   // Making sure we don't read before we have written.
   wire                         updated0;
   wire                         updated1;
   assign updated0 = first_stage?1:(readbuf_switch?updatedX[in0_addr]:updatedY[in0_addr]);
   assign updated1 = first_stage?1:(readbuf_switch?updatedX[in1_addr]:updatedY[in1_addr]);
   // Set to 1 when we want the twiddle factor module to return some new
   // twiddle factors.
   reg                          tf_addr_nd;
   // Tells the BF module that we are sending some data.
   reg                          x_nd;
   wire [2*TF_WDTH-1:0]         tf;

   initial
     begin
        fsm_state <= FSM_ST_INIT;
        tf_addr_nd <= 1'b0;
        x_nd <= 1'b0;
        readbuf_switch <= 1'b0;
        bufferin_read_switch <= 1'b0;
        bufferin_full0_B <= 1'b0;
        bufferin_full1_B <= 1'b0;
     end
   
   // Create the FSM machine
   always @ (posedge clk or negedge rst_n)
     begin
        if (!rst_n)
          begin
             fsm_state <= FSM_ST_INIT;
             tf_addr_nd <= 1'b0;
             x_nd <= 1'b0;
             readbuf_switch <= 1'b0;
             bufferin_read_switch <= 1'b0;
             bufferin_full0_B <= 1'b0;
             bufferin_full1_B <= 1'b0;
          end
        else
          begin
             // Delay for readbuf_switch.
             readbuf_switch_old <= readbuf_switch;
             // Take note of when new data arrives.
             case (fsm_state)
               FSM_ST_INIT:
                 begin
                    // Starting a new FFT (we may not have received input data
                    // yet but we can still prepare.
                    `MSG_DEBUG("FSM_ST_INIT");
			        out0_addr <= 0;
			        // For the first stage we write to (the second stage) there
                    // are N/2 series.
			        series_bits <= {NLOG2{1'b1}} >> 1;
			        // There are N/2 series in that stage.
			        S <= {1'b1,{NLOG2-1{1'b0}}};
                    // Tell twiddle factor module to calculate the first
                    //  twiddle factor.
                    tf_addr_nd <= 1'b1;
                    x_nd <= 1'b0;
                    finished <= 1'b0;
                    fsm_state <= FSM_ST_IDLE;
                 end // case: FSM_ST_INIT
               FSM_ST_IDLE:
                 begin
                    // Copy the input data into a buffer.
                    // If no input data is there we wait here until receiving
                    // input data.
                    // During the first step in this state the twiddle
                    // factor module will update the twiddle factor.
                    // During the last step in this state the BF module is
                    // sent it's first inputs.
                    `MSG_DEBUG("FSM_ST_IDLE");
                    tf_addr_nd <= 1'b0;
                    if (bufferin_read_full)
                      begin
                         fsm_state <= FSM_ST_CALC;
                         x_nd <= 1'b1;
                      end
                 end // case: FSM_ST_IDLE
               FSM_ST_CALC:
                 begin
                    `MSG_DEBUG("FSM_ST_CALC");
                    // In this state sections, series_bits, out0_addr and
                    // readbuf switch are updated so that we know where
                    // the BF module should read from and write to.
                    fsm_state <= FSM_ST_SEND;
                    tf_addr_nd <= 1'b1;
                    x_nd <= 1'b0;
			        if (&(out1_addr))
			          begin
				         // We finished the last FFT stage.  Move onto the next.
                         `MSG_DEBUG("-------NEXT STAGE---------");
                         // If we're on the first stage then free up the input buffer
                         // for more input.
                         if (first_stage)
                           begin
                              `MSG_DEBUG("-Input Buffer No Longer Full-");
                              if (bufferin_read_switch)
                                bufferin_full1_B <= ~bufferin_full1_B;
                              else
                                bufferin_full0_B <= ~bufferin_full0_B;
                              bufferin_read_switch <= ~bufferin_read_switch;
                           end
                         // One less bit of in0_addr corresponds to which section
                         // it is in.
				         series_bits <= series_bits >> 1;
                         // We have half as many sections as in the last stage.
				         S <= S >> 1;
				         out0_addr <= 0;
                         // We switch which buffers we are reading from and
                         // writing to.
                         readbuf_switch <= ~readbuf_switch;
                         // Mark the buffer we were previously reading from as
                         // not updated.  We will write to it now.
                         // Moved later so we drive from same process as we set.
                         /*
                         if (readbuf_switch)
                           updatedX <= {N{1'b0}};
                         else
                           updatedY <= {N{1'b0}};
                          */
			          end
			        else
			          begin
                         `MSG_DEBUG("-------NEXT POSITION---------");
				         // Otherwise we still have more sections to do at
                         // this position.
				         out0_addr <= out0_addr + 1;
			          end
                 end
               FSM_ST_SEND:
                 begin
                    `MSG_DEBUG("FSM_ST_SEND");
                    tf_addr_nd <= 1'b0;
                    // Wait in this state until the data we need to read is ready
                    // to go.
                    if (updated0 & updated1)
                      begin
                         x_nd <= 1'b1;
                         // If we have just sent data for the last BF calculation
                         // of the FFT calculation then go to the INIT state.
                         if (&(out1_addr) & (S==1))
                           begin
                              `MSG_DEBUG("--------FINISHED LAST STAGE---------");
                              fsm_state <= FSM_ST_INIT;
                              finished <= 1'b1;
                           end
                         else
                           fsm_state <= FSM_ST_CALC;
                      end
                    else
                      begin
                         `MSG_DEBUG("Waiting for data to be written.");
                      end
                 end
               default:
                 begin
                    fsm_state <= FSM_ST_INIT;
                 end
             endcase
          end              
     end
   
   /********************************************************/
   /*                                                      */
   /* Define logic that receives data from the BF.         */
   /*                                                      */
   /********************************************************/

   // Outputs from the BF
   // The addresses where the output should be written to.
   wire [NLOG2-1:0]         out0_addr_z;
   wire [NLOG2-1:0]         out1_addr_z;
   // The real and imag components of the output.
   wire [2*X_WDTH-1:0]      z;
   // Set to 1 when the ZA is output.
   // On the step after ZB is output.
   wire                     z_nd;
   // Set to 1 for the last ZA of a FFT.
   wire                     finished_z;
   // Set to 1 if data is from the last stage.
   wire                     last_stage_z;
   // Indicates which buffer the inputs to the BF module were
   // read from.
   // 1 if read from X.
   // 0 if read from Y.
   wire 					readbuf_switch_z;
   // Delayed content of readbuf_swith_z;
   reg                      readbuf_switch_z_last;
   // Delayed content of out1_addr_z_old since we need to use
   // it after it may have changed.
   reg [NLOG2-1:0]          out1_addr_z_old;
   // The address to write the currently received BF output.
   wire [NLOG2-1:0]         out_addr_z;
   assign out_addr_z = (z_nd)?out0_addr_z:out1_addr_z_old;
   // A delayed z_nd.  Tells us when to expect ZB.
   reg                      z_nd_last;
   // For delaying variables.  It takes 2 steps to write the output data
   // to the buffer at which point we decide whether to write the data
   // to bufferout.  These registers are needed for that decision.
   reg                      finished_z_old[1:0];
   reg                      last_stage_z_old[0:0];
   reg                      readbuf_switch_z_old[1:0];

   initial
     begin
        bufferout_full_A <= 1'b0;
        z_nd_last <= 1'b0;
     end
   
   always @ (posedge clk or negedge rst_n)
	 begin
		if (!rst_n)
		  begin
             bufferout_full_A <= 1'b0;
             z_nd_last <= 1'b0;
          end
        else
          begin
             // Put updated reset here so we drive it from same process.
             if ((fsm_state == FSM_ST_CALC) & (&(out1_addr)))
               begin
                  if (readbuf_switch)
                    updatedX <= {N{1'b0}};
                  else
                    updatedY <= {N{1'b0}};
               end
             // Set all the delays.
             readbuf_switch_z_last <= readbuf_switch_z;
             finished_z_old[0] <= finished_z;
             finished_z_old[1] <= finished_z_old[0];
             last_stage_z_old[0] <= last_stage_z;
             readbuf_switch_z_old[0] <= readbuf_switch_z;
             readbuf_switch_z_old[1] <= readbuf_switch_z_old[0];
             out1_addr_z_old <= out1_addr_z;
             z_nd_last <= z_nd;
             if (finished_z_old[1])
               // We have filled the output buffer
               bufferout_full_A <= ~bufferout_full_A;
             // Write received data to the buffers and set updated flag.
             if (z_nd | z_nd_last)
               begin
                  if ((last_stage_z & z_nd)|(last_stage_z_old[0] & ~z_nd))
                    begin
                       bufferout[out_addr_z] <= z;
                    end
		          else
                    begin
                       if ((readbuf_switch_z & z_nd)|(readbuf_switch_z_old[0] & ~z_nd))
                         begin
		                    bufferY[out_addr_z] <= z;
                            updatedY[out_addr_z] <= 1'b1;
                         end
		               else
                         begin
		                    bufferX[out_addr_z] <= z;
                            updatedX[out_addr_z] <= 1'b1;
                         end
                    end
               end
          end
     end

   /* Instantiate twiddle factor unit. */
   twiddlefactors
     twiddlefactors_0 (
                       .clk (clk),
                       .addr (tf_addr),
                       .addr_nd (tf_addr_nd),
                       .tf_out (tf)
                       );
   
   /* Instantiate the generic butterfly unit. */
   butterfly #(
		.M_WDTH   (3 + 2*NLOG2),
		.X_WDTH   (X_WDTH)
		)
   butterfly_0 (
				 .clk      (clk),
				 .rst_n    (rst_n),
				 .m_in     ({readbuf_switch_old, out0_addr, out1_addr, finished, last_stage}),
				 .w        (tf),
				 .xa       (in0),
				 .xb       (in1),
				 .x_nd     (x_nd),
				 .m_out    ({readbuf_switch_z, out0_addr_z, out1_addr_z, finished_z, last_stage_z}),
				 .y        (z),
				 .y_nd     (z_nd)
				 );

endmodule // dit

