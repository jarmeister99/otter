`timescale 1ns / 1ps


module testbench_wrapper();

logic CLK;
logic BTNC;
logic BTNL;
logic [7:0] SWITCHES;
logic [7:0] LEDS;
logic [7:0] CATHODES;
logic [3:0] ANODES;



initial begin
CLK = 1'B0;
BTNC = 1'B0;
BTNL = 1'b0;
SWITCHES = 8'B00000000;
end

OTTER_Wrapper_Programmable uut(.CLK(CLK), .BTNL(BTNL), .BTNC(BTNC), 
            .SWITCHES(SWITCHES), .LEDS(LEDS), .CATHODES(CATHODES), 
             .ANODES(ANODES));

always
 #10 CLK = ~CLK;






endmodule