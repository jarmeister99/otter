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
    input [4:0] DE_EX_RD,
    input [4:0] EX_MEM_RD,
    input [4:0] MEM_WB_RD,
    input [4:0] DE_RF_ADDR1,
    input [4:0] DE_RF_ADDR2,
    output logic STALL_IF,
    output logic STALL_DE
);
always_comb begin
    STALL_IF = 0;
    STALL_DE = 0;
    // If we decoded an instruction that attempts to access a register that is in the process of being written to
    if (DE_RF_ADDR1 == DE_EX_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR1 == EX_MEM_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR1 == MEM_WB_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR2 == DE_EX_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR2 == EX_MEM_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR2 == MEM_WB_RD) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
end
endmodule
