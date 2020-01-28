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
    input [31:0] DE_EX_RS1,
    input [31:0] DE_EX_RS2,
    input  [1:0] EX_MEM_PC_SEL,
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
    STALL_IF  <= 0;
    STALL_DE  <= 0;
    STALL_EX  <= 0;
    STALL_MEM <= 0;
    STALL_WB  <= 0;
    
    // If the first output (data) of the REG_FILE at the DECODE STAGE is equal to
    // ... the register to save data to in the EXECUTE STAGE 
    // Then stall IF, DE, EX
    if (EX_MEM_RD == DE_EX_RS1) begin
        STALL_IF <= 1;
        STALL_DE <= 1;
        STALL_EX <= 1;
    end
    // ...
    /// Stall IF, DE, EX
    if (EX_MEM_RD == DE_EX_RS2) begin
        STALL_IF <= 1;
        STALL_DE <= 1;
        STALL_EX <= 1;
    end
    // If the first output (data) of the REG_FILE at the DECODE STAGE is equal to
    // ... the register to save data to in the MEMORY STAGE
    // Then stall IF, DE, EX, MEM
    if (MEM_WB_RD == DE_EX_RS1) begin
        STALL_IF  <= 1;
        STALL_DE  <= 1;
        STALL_EX  <= 1;
        STALL_MEM <= 1;
    end
    if (MEM_WB_RD == DE_EX_RS2) begin
        STALL_IF  <= 1;
        STALL_DE  <= 1;
        STALL_EX  <= 1;
        STALL_MEM <= 1;
    end
end
endmodule
