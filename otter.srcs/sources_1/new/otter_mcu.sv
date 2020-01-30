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
    logic [2:0] func3;
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
    logic [31:0] rs1, rs2;
    logic [31:0] pc, ir;
} instr_t;

module otter_mcu(
    input CLK,
    input RESET,
    input  [31:0] IOBUS_IN,
    output logic  [31:0] IOBUS_ADDR,
    output logic [31:0] IOBUS_OUT,
    output logic IOBUS_WR
);

// PIPELINE CONTROLS
logic stallIf, stallDe;

// ~~~~~~~~~~~ //
// FETCH STAGE //
// ~~~~~~~~~~~ //

wire  [31:0] pc, nextPc, ir;
instr_t if_de_inst;

initial begin
    if_de_inst.pc = 0;
end

always_ff @(posedge CLK) begin
    if (!stallIf) begin
        if_de_inst.pc <= pc; // If the STALL signal is asserted, the IF_DE register should not update
    end
end

// ~~~~~~~~~~~~ //
// DECODE STAGE //
// ~~~~~~~~~~~~ //

wire  [31:0] aluAIn, aluBIn; // The outputs from the 2to1 and 4to1 RF mux's
wire  [31:0] sTypeImmed, iTypeImmed, uTypeImmed;
wire  [31:0] rs1, rs2;
wire   [3:0] aluFun;
wire   [1:0] rfWrSel;
wire   [1:0] aluSrcB;
wire         aluSrcA;

instr_t de_inst;
opcode_t opcode;

instr_t de_ex_inst;
logic [31:0] de_ex_iTypeImmed;
logic [31:0] de_ex_aluAIn, de_ex_aluBIn;

// Combinatorially populate de_inst structure with information from the instruction, and other combinatorial modules in the decode stage
always_comb begin
    de_inst.ir       = ir;
    de_inst.pc       = if_de_inst.pc;
    
    opcode           = opcode_t'(de_inst.ir[6:0]);
    de_inst.memWrite = opcode == STORE;
    de_inst.memRead2 = opcode == LOAD;    // Make sure this is the right ir
    de_inst.regWrite = de_inst.ir[6:0] != BRANCH &&
                       de_inst.ir[6:0] != LOAD   &&           
                       de_inst.ir[6:0] != STORE; 
    de_inst.rd       = de_inst.ir[11:7];
    de_inst.rfAddr1  = de_inst.ir[19:15];
    de_inst.rfAddr2  = de_inst.ir[24:20];
    de_inst.func3    = de_inst.ir[14:12]; // func3 is also memType
    de_inst.opcode   = opcode;
    de_inst.aluFun   = aluFun;     // received from decoder
    de_inst.rfWrSel  = rfWrSel;    // received from decoder
    de_inst.rs1      = rs1;        // received from reg
    de_inst.rs2      = rs2;        // received from reg
    
end
always_ff @(posedge CLK) begin
    // Save decoded instructions to DE_EX register if appropriate
    if (!stallDe) begin
        de_ex_inst <= de_inst;
        de_ex_aluAIn     <= aluAIn;
        de_ex_aluBIn     <= aluBIn;
        de_ex_iTypeImmed <= iTypeImmed;
    end
    else begin
        de_ex_inst       <= 0;
        de_ex_aluAIn     <= 0;
        de_ex_aluBIn     <= 0;
        de_ex_iTypeImmed <= 0;
    end
end

// ~~~~~~~~~~~~~ //
// EXECUTE STAGE //
// ~~~~~~~~~~~~~ //

wire [31:0] jalrPc, branchPc, jalPc, seqPc;    
wire [31:0] aluRes; 
wire  [1:0] pcSel;                    
instr_t ex_mem_inst;
logic [31:0] ex_mem_aluRes;


always_ff @(posedge CLK) begin
    ex_mem_inst     <= de_ex_inst;
    ex_mem_aluRes   <= aluRes; // If the DE_EX stage has been invalidated, future stages must be no-ops
end



// ~~~~~~~~~~~~ //
// MEMORY STAGE //
// ~~~~~~~~~~~~ //
            
wire [31:0] memOut2;
instr_t mem_wb_inst;
logic [31:0] mem_wb_aluRes;

always_ff @(posedge CLK) begin
    mem_wb_inst     <= ex_mem_inst;
    mem_wb_aluRes   <= ex_mem_aluRes; // If the EX_MEM stage has been invalidated, future stages must be no-ops
end 

always_comb begin
    IOBUS_OUT = ex_mem_inst.rs2;
    IOBUS_ADDR = ex_mem_aluRes;
end

// ~~~~~~~~~~~~~~~ //
// WRITEBACK STAGE //
// ~~~~~~~~~~~~~~~ //

wire  [31:0] memData;   
wire  [31:0] dataToRegWrite;

// PIPELINE CONTROL LOGIC


// MODULES

ProgCount prog_count(
    .PC_CLK    (CLK),    
    .PC_RST    (RESET),
    .PC_LD     (~stallIf),  // If the STALL signal is asserted, the PC should not update
    .PC_DIN    (nextPc),     
    .PC_COUNT  (pc)
);

Mult4to1 prog_count_next_mux(
    .In1  (if_de_inst.pc + 4),      // Option 1: Current PC + 4
    .In2  (jalrPc),      // Option 2: JALR TARGET calculated during EX STAGE
    .In3  (branchPc),    // Option 3: BRANCH TARGET calculated during EX STAGE
    .In4  (jalPc),       // Option 4: JAL TARGET calculated during EX STAGE
    .Sel  (pcSel),       // Determined via PC_SEL calculated during EX STAGE
    .Out  (nextPc)              
);    

// The MEM is used to store or access data, including the current instruction.
// -- Its first address
OTTER_mem_byte mem(
    .MEM_CLK     (CLK),
    .MEM_ADDR1   (pc),              
    .MEM_ADDR2   (ex_mem_aluRes),             // Access memory at the location corresponding to the ALU RESULT during the MEM STAGE 
    .MEM_DIN2    (ex_mem_inst.rs2),           // Save the value from register 2 during the MEM STAGE
    .MEM_WRITE2  (ex_mem_inst.memWrite),     
    .MEM_READ1   (~stallIf),                  // An instruction can only be fetched from MEM1 if STALL_FETCH signal is not given
    .MEM_READ2   (ex_mem_inst.memRead2),      // MEM2 can only be read from if command is LOAD
    .IO_IN       (IOBUS_IN),                 
    .MEM_SIZE    (ex_mem_inst.func3[1:0]),
    .MEM_SIGN    (ex_mem_inst.func3[2]),
    .ERR         (),
    .MEM_DOUT1   (ir),                        // Output of MEM1 is the current instruction
    .MEM_DOUT2   (memOut2),                   // Output of MEM2 is the requested data
    .IO_WR       (IOBUS_WR)
);

OTTER_CU_Decoder decoder(
    .CU_OPCODE     (de_inst.ir[6:0]),      // Given from IR @ DECODE STAGE
    .CU_FUNC3      (de_inst.ir[14:12]),    // Given from IR @ DECODE STAGE
    .CU_FUNC7      (de_inst.ir[31:25]),    // Given from IR @ DECODE STAGE
    .CU_ALU_SRCA   (aluSrcA),            // Used for DECODE STAGE logic
    .CU_ALU_SRCB   (aluSrcB),            // Used for DECODE STAGE logic
    .CU_ALU_FUN    (aluFun),             // Used for EXECUTE STAGE logic
    .CU_RF_WR_SEL  (rfWrSel)             // Used for WRITEBACK STAGE logic
);

OTTER_registerFile reg_file(
    .READ1         (de_inst.rfAddr1),        
    .READ2         (de_inst.rfAddr2),      
    .DEST_REG      (mem_wb_inst.rd),       
    .DIN           (dataToRegWrite),        
    .WRITE_ENABLE  (mem_wb_inst.regWrite),  
    .OUT1          (rs1),                   
    .OUT2          (rs2),                    
    .CLK           (CLK)
);
Mult4to1 reg_file_data_mux(
    .In1  (mem_wb_inst.pc + 4),             // Option 1: PC from WRITEBACK STAGE + 4
    .In2  (0),                         // Option 2: NONE
    .In3  (memOut2),            // Option 3: Output of MEM1 from WRITEBACK STAGE
    .In4  (mem_wb_aluRes),             // Option 4: Output of ALU from WRITEBACK STAGE
    .Sel  (mem_wb_inst.rfWrSel),     
    .Out  (dataToRegWrite)
);

OTTER_ALU alu(
    .ALU_FUN  (de_ex_inst.aluFun),  // Given from inst package @ EXECUTE STAGE
    .A        (de_ex_aluAIn),       
    .B        (de_ex_aluBIn),
    .ALU_OUT  (aluRes)
);
Mult2to1 alu_ain_mux(
    .In1  (de_inst.rs1),
    .In2  (uTypeImmed),
    .Sel  (aluSrcA),
    .Out  (aluAIn)
);
Mult4to1 alu_bin_mux(
    .In1  (de_inst.rs2),
    .In2  (iTypeImmed),
    .In3  (sTypeImmed),
    .In4  (de_inst.pc),
    .Sel  (aluSrcB),
    .Out  (aluBIn)
);
    
target_gen target_gen(
    .RS1        (de_ex_inst.rs1),            
    .I_IMMED    (de_ex_iTypeImmed), 
    .PC         (de_ex_inst.pc),   
    .IR         (de_ex_inst.ir),     
    .JALR_PC    (jalrPc),
    .BRANCH_PC  (branchPc),
    .JUMP_PC    (jalPc)
);
immed_gen immed_gen(
    .IR            (de_inst.ir),
    .S_TYPE_IMMED  (sTypeImmed),
    .I_TYPE_IMMED  (iTypeImmed),
    .U_TYPE_IMMED  (uTypeImmed)
);
branch_cond_gen branch_cond_gen(
    .A       (de_ex_aluAIn),         
    .B       (de_ex_aluBIn),          
    .OPCODE  (de_ex_inst.opcode),
    .FUNC3   (de_ex_inst.func3),
    .PC_SEL  (pcSel)
);

// CHECK INPUTS, MAYBE WRONG? //
hazard_detector hazard_detector(
);


endmodule
