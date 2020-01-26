`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/26/2020 01:43:42 PM
// Design Name: 
// Module Name: branch_cond_gen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module branch_cond_gen(
    input  [31:0] A,
    input  [31:0] B,
    output logic BR_LT,
    output logic BR_EQ,
    output logic BR_LTU
);

always_comb begin
    BR_LTU=0; BR_EQ=0; BR_LTU=0;
    if($signed(A) < $signed(B)) BR_LT=1;
    if(A==B)                    BR_EQ=1;
    if(A<B)                     BR_LTU=1;    
end

endmodule
