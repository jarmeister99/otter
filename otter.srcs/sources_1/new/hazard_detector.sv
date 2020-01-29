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
    input [4:0] EX_MEM_RD,
    input [4:0] MEM_WB_RD,
    input [4:0] DE_EX_RF_ADDR1,
    input [4:0] DE_EX_RF_ADDR2,
    input [1:0] EX_MEM_PC_SEL,
    output logic STALL_IF,
    output logic STALL_DE,
    output logic INVALIDATE
);
always_comb begin
    

    if (EX_MEM_PC_SEL != 0) begin
        INVALIDATE = 1;
    end
    else begin
        INVALIDATE = 0;
    end
    
    // If the first output (data) of the REG_FILE at the DECODE STAGE is equal to
    // ... the register to save data to in the EXECUTE STAGE 
    // Then stall IF, DE
    if (EX_MEM_RD == DE_EX_RF_ADDR1) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    else begin
        STALL_IF = 0;
        STALL_DE = 0;
    end
    // ...
    /// Stall IF, DE
    if (EX_MEM_RD == DE_EX_RF_ADDR2) begin
        STALL_IF = 1;
        STALL_DE = 1;
    end
    else begin
        STALL_IF = 0;
        STALL_DE = 0;
    end
    // If the first output (data) of the REG_FILE at the DECODE STAGE is equal to
    // ... the register to save data to in the MEMORY STAGE
    // Then stall IF, DE
    if (MEM_WB_RD == DE_EX_RF_ADDR1) begin
        STALL_IF  = 1;
        STALL_DE  = 1;
    end
    else begin
        STALL_IF  = 0;
        STALL_DE  = 0;
    end
    if (MEM_WB_RD == DE_EX_RF_ADDR2) begin
        STALL_IF  = 1;
        STALL_DE  = 1;
    end
    else begin
        STALL_IF  = 0;
        STALL_DE  = 0;
    end
end
endmodule
