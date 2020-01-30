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
    input [1:0] EX_PC_SEL,
    input [4:0] DE_EX_RD,
    input [4:0] EX_MEM_RD,
    input [4:0] MEM_WB_RD,
    input [4:0] DE_RF_ADDR1,
    input [4:0] DE_RF_ADDR2,
    output logic STALL_IF,
    output logic STALL_DE,
    output logic INVALIDATE
);
typedef enum logic [6:0] {
    LUI      = 7'b0110111,
    AUIPC    = 7'b0010111,
    JAL      = 7'b1101111,
    JALR     = 7'b1100111,
    BRANCH   = 7'b1100011,
    LOAD     = 7'b0000011,
    STORE    = 7'b0100011,
    OP_IMM   = 7'b0010011,
    OP       = 7'b0110011,
    SYSTEM   = 7'b1110011
} opcode_t;

logic branchActive;

always_comb begin
    INVALIDATE = 0;
    STALL_IF   = 0;
    STALL_DE   = 0;
    
    if (EX_PC_SEL != 0) begin
        INVALIDATE = 1;
    end
    
    // If we decoded an instruction that attempts to access a register that is in the process of being written to
    if (DE_RF_ADDR1 == DE_EX_RD && DE_RF_ADDR1 != 0) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR1 == EX_MEM_RD && DE_RF_ADDR1 != 0) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    if (DE_RF_ADDR1 == MEM_WB_RD && DE_RF_ADDR1 != 0) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
//    if (DE_RF_ADDR2 == DE_EX_RD) begin    ~~~~~~~~~~~~~~~~~~~|| THINK ABOUT THIS MORE ||~~~~~~~~~~~~~~~~~~~
//        STALL_IF = 1;
//        STALL_DE = 1;
//    end
//    if (DE_RF_ADDR2 == EX_MEM_RD) begin
//        STALL_IF = 1;
//        STALL_DE = 1;
//    end
//    if (DE_RF_ADDR2 == MEM_WB_RD) begin
//        STALL_IF = 1;
//        STALL_DE = 1;
//    end
end
endmodule
