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
CLK = 0;
BTNC = 0;
BTNL = 0;
SWITCHES = 0;
end

OTTER_Wrapper_Programmable uut(.CLK(CLK), .BTNL(BTNL), .BTNC(BTNC), 
            .SWITCHES(SWITCHES), .LEDS(LEDS), .CATHODES(CATHODES), 
             .ANODES(ANODES));

always
 #10 CLK = ~CLK;






endmodule