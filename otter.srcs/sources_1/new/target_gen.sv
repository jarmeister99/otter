`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2020 09:09:33 AM
// Design Name: 
// Module Name: target_gen
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


module target_gen(
    input  [31:0] RS1,         // A, aka the output from reg_file port 1
    input  [31:0] I_IMMED,
    input  [31:0] PC,
    input  [31:0] IR,
    output [31:0] JALR_PC,
    output [31:0] BRANCH_PC,
    output [31:0] JUMP_PC
);

assign JALR_PC   = I_IMMED + RS1;
assign BRANCH_PC = PC + {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
assign JUMP_PC   = PC + {{12{IR[31]}}, IR[19:12], IR[20], IR[30:21], 1'b0};

endmodule
