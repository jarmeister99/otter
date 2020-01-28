`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/27/2020 04:43:06 PM
// Design Name: 
// Module Name: hazard_detector
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


module hazard_detector(
    input CLK,
    input [31:0] EX_MEM_RD,
    input [31:0] MEM_WB_RD,
    input [31:0] ID_EX_RS1,
    input [31:0] ID_EX_RS2,
    output logic STALL_IF,
    output logic STALL_DE,
    output logic STALL_EX,
    output logic STALL_MEM,
    output logic STALL_WB,
    output logic INVALID_IF,
    output logic INVALID_DE,
    output logic INVALID_EX,
    output logic INVALID_MEM,
    output logic INVALID_WB
);
always_ff @(posedge CLK) begin
    if (EX_MEM_RD == ID_EX_RS1) begin
    end
    if (EX_MEM_RD == ID_EX_RS2) begin
    end
    if (MEM_WB_RD == ID_EX_RS1) begin
    end
    if (MEM_WB_RD == ID_EX_RS2) begin
    end
end
endmodule
