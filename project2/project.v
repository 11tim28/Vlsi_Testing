module lfsr16(
   input             clk,
   input             rst,
   input             enable,
   output   [15:0]   state,
   output   [3:0]    probs_out
);
   reg   [15:0]   lfsr_r, lfsr_w;

   // primitive polynomial x^16 + x^5 + x^4 + x^3 + 1
   wire feedback = lfsr_r[5] ^ lfsr_r[4] ^ lfsr_r[3] ^ lfsr_r[0];
   assign state = lfsr_r;
   assign probs_out = {lfsr_r[15], lfsr_r[10], lfsr_r[5], lfsr_r[0]};

   always@(*) begin
      if(enable)  lfsr_w   =  {feedback, lfsr_r[15:1]};
      else        lfsr_w   =  lfsr_r;
   end

   always@(posedge clk or posedge rst) begin
      if(rst) begin
         lfsr_r   <= 16'hACE1; // seed
      end
      else begin
         lfsr_r   <= lfsr_w;
      end
   end

endmodule

module misr16(
   input             clk,
   input             rst,
   input             enable,
   input    [3:0]    data_in,
   output   [15:0]   signature
);
   reg   [15:0]   misr, misr_w;

   wire  feedback    =  misr[5] ^ misr[4] ^ misr[3] ^ misr[0];

   assign signature  =  misr;

   always@(*) begin
      if(enable)  misr_w   = {feedback, misr[15] ^ data_in[3], misr[14] ^ data_in[2], misr[13] ^ data_in[1], misr[12] ^ data_in[0], misr[11:1]};
      else        misr_w   =  misr;
   end

   always@(posedge clk or posedge rst) begin
      if(rst) begin
         misr  <= 16'd0;
      end
      else begin
         misr  <= misr_w;
      end
   end

endmodule



module bist_hardware(clk,rst,bistmode,bistpass,bistdone,cut_scanmode,
                     cut_sdi,cut_sdo);
  input          clk;
  input          rst;
  input          bistmode;
  output         bistpass;
  output         bistdone;
  output         cut_scanmode;
  output  [3:0]  cut_sdi;
  input   [3:0]  cut_sdo;

// Add your code here, below is just temp code so testbench doesn't hang

parameter S_IDLE = 3'd0;
parameter S_SHFT = 3'd1;
parameter S_CPTR = 3'd2;
parameter S_DONE = 3'd3;
// parameter S_DONE = 4;
parameter SIG_GLDN = 16'h0000;

// Test git
// Scan Chain length:
// 1: 56, 2: 57, 3: 57, 4: 57
   reg   [2:0]    state_r, state_w;
   reg   [11:0]   pttrn_cnt_r, pttrn_cnt_w;
   reg   [7:0]    scan_cnt_r, scan_cnt_w;
   reg            lfsr_en_r, misr_en_r, cut_scanmode_r;
   reg            pass_r, done_r;

   wire           lfsr_en;
   wire           misr_en;
   wire  [15:0]   lfsr_state;
   wire  [15:0]   misr_sig;

   assign lfsr_en       =  lfsr_en_r;
   assign misr_en       =  misr_en_r;
   assign cut_scanmode  =  cut_scanmode_r;
   assign bistpass      =  pass_r;
   assign bistdone      =  done_r;

   lfsr16 lfsr_m(
      .clk(clk),
      .rst(rst),
      .enable(lfsr_en),
      .state(lfsr_state),
      .probs_out(cut_sdi)
   );

   misr16 misr_m(
      .clk(clk),
      .rst(rst),
      .enable(misr_en),
      .data_in(cut_sdo),
      .signature(misr_sig)
   );


   always@(*) begin
      case(state_r)
         S_IDLE: begin
            lfsr_en_r         =  1'b0;
            misr_en_r         =  1'b0;
            cut_scanmode_r    =  1'b0;
            pass_r            =  1'b0;
            done_r            =  1'b0;
            pttrn_cnt_w       =  12'd0;
            scan_cnt_w        =  8'd0;
            if(rst && bistmode)  state_w  =  S_SHFT;
            else                 state_w  =  state_r;
         end

         S_SHFT: begin
            pass_r               =  1'b0;
            done_r               =  1'b0;
            cut_scanmode_r       =  1'b1;
            if(pttrn_cnt_r < 12'd3000) begin
               lfsr_en_r         =  1'b1;
               if(scan_cnt_r < 8'd57) begin
                  scan_cnt_w     =  scan_cnt_r  +  8'd1;
                  state_w        =  state_r;
               end
               else begin
                  scan_cnt_w     =  8'd0;
                  state_w        =  S_CPTR;
               end
            end
            else begin
               // Shift out last response
               lfsr_en_r         =  1'b0;
               if(scan_cnt_r < 8'd57) begin
                  scan_cnt_w     =  scan_cnt_r  +  8'd1;
                  state_w        =  state_r;
               end
               else begin
                  scan_cnt_w     =  8'd0;
                  state_w        =  S_DONE;
               end
            end

            if(pttrn_cnt_r > 12'd0) misr_en_r   =  1'b1;
            else                    misr_en_r   =  1'b0;

         end

         S_CPTR: begin
            misr_en_r         =  1'b0;
            pass_r            =  1'b0;
            done_r            =  1'b0;
            lfsr_en_r         =  1'b0;
            cut_scanmode_r    =  1'b0;
            pttrn_cnt_w       =  pttrn_cnt_r + 12'd1;
            scan_cnt_w        =  8'd0;
            state_w           =  S_SHFT;
         end

         S_DONE: begin
            done_r            =  1'b1;
            lfsr_en_r         =  1'b0;
            cut_scanmode_r    =  1'b0;
            misr_en_r         =  1'b0;
            $display("MISR Signature: %h", misr_sig);
            if(misr_sig == SIG_GLDN)   pass_r   =  1'b1;
            else                       pass_r   =  1'b0;
            state_w     =  S_IDLE;
         end

         default: begin
            misr_en_r         =  1'b0;
            pass_r            =  1'b0;
            done_r            =  1'b0;
            lfsr_en_r         =  1'b0;
            cut_scanmode_r    =  1'b0;
            state_w     =  S_IDLE;
         end
      endcase
   end

   always@(posedge clk or posedge rst) begin
      if(rst) begin
         state_r        <=    state_w;
         pttrn_cnt_r    <=    12'd0;
         scan_cnt_r     <=    8'd0;
      end
      else begin
         state_r        <=    state_w;
         pttrn_cnt_r    <=    pttrn_cnt_w;
         scan_cnt_r     <=    scan_cnt_w;
      end
   end


   
endmodule


module chip(clk,rst,pi,po,bistmode,bistdone,bistpass);
   input         clk;
   input 	 rst;
   input  [35:0] pi;
   output [48:0] po;
   input 	 bistmode;
   output 	 bistdone;
   output 	 bistpass;

   wire 	 cut_scanmode;
   wire [3:0] 	 cut_sdi,cut_sdo;


   scan_cut circuit(bistmode,cut_scanmode,clk,rst,cut_sdi, pi, po, cut_sdo);
   bist_hardware bist( clk,rst,bistmode,bistpass,bistdone,cut_scanmode,
                       cut_sdi,cut_sdo);
  
endmodule
