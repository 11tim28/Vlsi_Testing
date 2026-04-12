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

// start temp code
   reg 		 bistdone;

   assign bistpass = 1;
   
   always @(posedge clk)
     if ( rst)
        bistdone = 0;
     else bistdone = 1;
//  end temp code
   
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
