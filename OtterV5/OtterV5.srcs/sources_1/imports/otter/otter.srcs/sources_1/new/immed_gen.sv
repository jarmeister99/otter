`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2020 09:44:21 AM
// Design Name: 
// Module Name: immed_gen
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


module immed_gen(
    input  [31:0] IR,
    output [31:0] S_TYPE_IMMED,
    output [31:0] I_TYPE_IMMED,
    output [31:0] U_TYPE_IMMED
);

assign S_TYPE_IMMED = {{20{IR[31]}}, IR[31:25], IR[11:7]};
assign I_TYPE_IMMED = {{20{IR[31]}}, IR[31:20]};
assign U_TYPE_IMMED = {IR[31:12], {12{1'b0}}};

endmodule
