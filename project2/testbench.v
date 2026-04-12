//`timescale 1ns/10ps 
`define NULL 0 
`define EOF 32'hFFFF_FFFF 
`define FAULTFREE 1 
`define BISTTEST 2 

module TOP_ctl ; 
   
 integer c, r, file_in, file_out; 
 integer mode; 
 integer detected_faults;
 integer faults;

   //Input registers
 reg [35:0] _pi;       //PI
 reg [0:0] _clk;       //clock
 reg [0:0] _rst;       //Reset
 reg [0:0] _bistmode;  //Bistmode

   //Output wires
 wire [0:0] _bistpass; //Bistpass
 wire [0:0] _bistdone; //Bistdone
 wire [48:0] _cut_po;  //PO

 parameter cycle = 4; //cycle time

   //Wires for external interface
wire pin_1_, pin_2_, pin_3_, pin_4_, pin_5_, pin_6_,
   pin_7_, pin_8_, pin_9_, pin_10_, pin_11_, pin_12_, pin_13_,
   pin_14_, pin_15_, pin_16_, pin_17_, pin_18_, pin_19_, pin_20_,
   pin_21_, pin_22_, pin_23_, pin_24_, pin_25_, pin_26_, pin_27_,
   pin_28_, pin_29_, pin_30_, pin_31_, pin_32_, pin_33_, pin_34_,
   pin_35_, pin_36_, pin_37_, pin_38_, pin_39_, pin_40_, pin_41_,
   pin_42_, pin_43_, pin_44_, pin_45_, pin_46_, pin_47_, pin_48_,
   pin_49_, pin_50_, pin_51_, pin_52_, pin_53_, pin_54_, pin_55_,
   pin_56_, pin_57_, pin_58_, pin_59_, pin_60_, pin_61_, pin_62_,
   pin_63_, pin_64_, pin_65_, pin_66_, pin_67_, pin_68_, pin_69_,
   pin_70_, pin_71_, pin_72_, pin_73_, pin_74_, pin_75_, pin_76_,
   pin_77_, pin_78_, pin_79_, pin_80_, pin_81_, pin_82_, pin_83_,
   pin_84_, pin_85_, pin_86_, pin_87_, pin_88_, pin_89_, pin_90_, pin_91_;


assign _bistdone = {pin_1_};
assign _bistpass = {pin_2_};
assign {pin_3_} = _bistmode;
assign {pin_4_} = _clk;
assign {pin_5_} = _rst;
assign {pin_6_, pin_7_,
   pin_8_, pin_9_, pin_10_, pin_11_, pin_12_, pin_13_, pin_14_,
   pin_15_, pin_16_, pin_17_, pin_18_, pin_19_, pin_20_, pin_21_,
   pin_22_, pin_23_, pin_24_, pin_25_, pin_26_, pin_27_, pin_28_,
   pin_29_, pin_30_, pin_31_, pin_32_, pin_33_, pin_34_, pin_35_,
   pin_36_, pin_37_, pin_38_, pin_39_, pin_40_, pin_90_, pin_91_ } = _pi;

assign _cut_po = { pin_41_, pin_42_,
   pin_43_, pin_44_, pin_45_, pin_46_, pin_47_, pin_48_, pin_49_,
   pin_50_, pin_51_, pin_52_, pin_53_, pin_54_, pin_55_, pin_56_,
   pin_57_, pin_58_, pin_59_, pin_60_, pin_61_, pin_62_, pin_63_,
   pin_64_, pin_65_, pin_66_, pin_67_, pin_68_, pin_69_, pin_70_,
   pin_71_, pin_72_, pin_73_, pin_74_, pin_75_, pin_76_, pin_77_,
   pin_78_, pin_79_, pin_80_, pin_81_, pin_82_, pin_83_, pin_84_,
   pin_85_, pin_86_, pin_87_, pin_88_, pin_89_ };


   // instantiate chip here
   chip testchip(
   	.clk(pin_4_),
	.rst(pin_5_),
        .pi(_pi),
	.po(_cut_po),
   	.bistmode( pin_3_ ),
	.bistdone( pin_1_ ),
	.bistpass( pin_2_ )
   );
   	
   always #(cycle / 2) _clk = ~_clk; //clock generator

//BIST task
task dobist;
begin
   $display("Starting BIST run");
   
   mode = `BISTTEST;            //set mode

   @(negedge _clk) begin        //Reset=1, enable BIST
   	_rst = 1;
   	_bistmode = 1;
   end

   #(cycle-1) _rst = 0;

   @(_bistdone == 1) begin      //When Bist done and bistpass status
      $write( "Completed Bist...  BIST Passed: %b\n", _bistpass );
      if(_bistpass == 0)
	 detected_faults = detected_faults+1;
   end
end
endtask

initial begin
   _clk = 1'b0;
   detected_faults = 0;
   faults = 0;

   //Fault Injection
   $write( "\nInjecting No Fault into CUT:  Fault-Free BIST Simulation\n" );
   dobist;

   force testchip.circuit.II282 = 1;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.II282 = 1\n" );
   faults = faults+1;
   dobist;
   release testchip.circuit.II282;

   force testchip.circuit.n482gat = 0;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n482gat = 0\n" );
   faults = faults+1;
   dobist;
   release testchip.circuit.n482gat;

   force testchip.circuit.n226gat = 1;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n226gat = 1\n" );
   faults = faults+1;
   dobist;
   release testchip.circuit.n226gat;

   force testchip.circuit.n1275gat = 0;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n1275gat = 0\n" );
   faults = faults+1;
   dobist;
   release testchip.circuit.n1275gat;

   force testchip.circuit.n1582gat = 1;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n1582gat = 1\n" );
   faults = faults+1;
   dobist;
   release testchip.circuit.n1582gat;

   force testchip.circuit.II4020 = 0;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.II4020 = 0\n " );
   faults = faults+1;
   dobist;
   release testchip.circuit.II4020;

   force testchip.circuit.n745gat = 1;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n745gat = 1\n " );
   faults = faults+1;
   dobist;
   release testchip.circuit.n745gat;

   force testchip.circuit.n1794gat = 0;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.n1794gat = 0\n " );
   faults = faults+1;
   dobist;
   release testchip.circuit.n1794gat;

   force testchip.circuit.II4623 = 1;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.II4623 = 1\n " );
   faults = faults+1;
   dobist;
   release testchip.circuit.II4623;

   force testchip.circuit.II4759 = 0;
   $write( "\nInjecting Fault into CUT:  testchip.circuit.II4759 = 0\n " );
   faults = faults+1;
   dobist;
   release testchip.circuit.II4759;

   $write("\nInjected %d faults\n", faults);
   $write("Detected %d faults\n", detected_faults);

   $finish;
end // initial begin

   
endmodule // TOP_ctl


