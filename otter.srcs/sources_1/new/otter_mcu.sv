`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2020 08:46:53 AM
// Design Name: 
// Module Name: otter_mcu
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

typedef struct packed{
    opcode_t opcode;
    logic [4:0] rfAddr1;
    logic [4:0] rfAddr2;
    logic [4:0] rd;
    logic rs1Used;
    logic rs2Used;
    logic rdUsed;
    logic [3:0] aluFun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rfWrSel;
    logic [2:0] memType;  //sign, size
    logic [31:0] rs2;
} instr_t;

module otter_mcu(
    input CLK,
    input RESET,
    input [31:0] IOBUS_IN,
    input [31:0] IOBUS_OUT,
    input [31:0] IOBUS_ADDR,
    output logic IOBUS_WR
);

// PIPELINE CONTROLS
logic stallIf=0, stallDe=0, stallEx=0, stallMem=0, stallWb=0; 

// FETCH STAGE
wire  [31:0] pc, nextPc;
wire  [31:0] rs1;
logic [1:0]  pcSel;

// DECODE STAGE
wire [31:0] sTypeImmed, iTypeImmed, uTypeImmed;

// EXECUTE STAGE

logic [31:0] de_ex_pc=0;

always_ff @(posedge CLK) begin
    if (!stallDe) begin
        de_ex_pc <= pc;
    end
end

// MEMORY STAGE
logic [31:0] jalrPc, branchPc, jalPc;     // Set by Target Generator

logic [1:0] ex_mem_pcSel=0;               // Set by Branch Condition Generator
assign pcSel = ex_mem_pcSel;

ProgCount prog_count(
    .PC_CLK(CLK),    
    .PC_RST(RESET),
    .PC_LD(~stallIf),  // Always 1 unless we should stall
    .PC_DIN(nextPc),   // Output of PC 4-1 MUX
    .PC_COUNT(pc)
);
Mult4to1 prog_count_next_mux(
    .In1(pc + 4),      // Default value 
    .In2(jalrPc),      // Calculated at EX/MEM stage
    .In3(branchPc),    // ...
    .In4(jalPc),       // ...
    .Sel(pcSel),       // Control signal set by EX/MEM stage, otherwise 0
    .Out(nextPc)
);    
    
target_gen target_gen(
    .RS1(rs1),            
    .I_IMMED(iTypeImmed), // iTypeImmed received from Immediate Generator
    .PC(de_ex_pc),        // Target generator must used saved PC
    .IR(ir),              // Current ir can be used here?
    .JALR_PC(jalrPc),
    .BRANCH_PC(branchPc),
    .JAL_C(jalPc)
);
    
immed_gen immed_gen(
    .IR(ir),
    .S_TYPE_IMMED(sTypeImmed),
    .I_TYPE_IMMED(iTypeImmed),
    .U_TYPE_IMMED(uTypeImmed)
);

endmodule
