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
    logic [2:0] memType;  //sign, size
    logic [31:0] rs1, rs2;
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
logic stallIf=0,   stallDe=0,   stallEx=0,   stallMem=0,   stallWb=0; 
logic invalidIf=0, invalidDe=0, invalidEx=0, invalidMem=0, invalidWb=0;

// ~~~~~~~~~~~ //
// FETCH STAGE //
// ~~~~~~~~~~~ //

wire  [31:0] pc, nextPc;

// ~~~~~~~~~~~~ //
// DECODE STAGE //
// ~~~~~~~~~~~~ //

wire  [31:0] sTypeImmed, iTypeImmed, uTypeImmed;
wire  [31:0] ir;         // May need to be stage reg?
wire  [31:0] rs1, rs2;
wire   [3:0] aluFun;
wire   [1:0] rfWrSel;
wire   [1:0] aluSrcB;
wire         aluSrcA;
logic [31:0] if_de_pc=0;
logic [31:0] if_de_ir=0;

instr_t if_de_inst;
opcode_t opcode;

always_ff @(posedge CLK) begin
    if (!stallDe) begin
        if_de_pc            <= pc;       // May need to happen in fetch stage
        if_de_ir            <= ir;
        if_de_inst.opcode   <= opcode;
        if_de_inst.aluFun   <= aluFun;
        if_de_inst.rfWrSel  <= rfWrSel;
        if_de_inst.rs1      <= rs1;
        if_de_inst.rs2      <= rs2;
     
    end
end

always_comb begin
    opcode              = opcode_t'(ir[6:0]);
    if_de_inst.rfAddr1  = ir[19:15];
    if_de_inst.rfAddr2  = ir[24:20];
    if_de_inst.func3    = ir[14:12];
    if_de_inst.memType  = ir[14:12];
    if_de_inst.rd       = ir[11:7];
    if_de_inst.memWrite = ir[6:0] == STORE;
    if_de_inst.memRead2 = ir[6:0] == LOAD;
    if_de_inst.regWrite = ir[6:0] != BRANCH &&
                          ir[6:0] != LOAD   &&            // How does this make sense?
                          ir[6:0] != STORE; 
end
    



// ~~~~~~~~~~~~~ //
// EXECUTE STAGE //
// ~~~~~~~~~~~~~ //

wire  [31:0] aluAIn, aluBIn;
logic [31:0] de_ex_iTypeImmed;
logic [31:0] de_ex_ir=0;
logic [31:0] de_ex_pc=0;
logic [31:0] de_ex_aluAIn=0, de_ex_aluBIn=0;
instr_t de_ex_inst;

always_ff @(posedge CLK) begin
    if (!stallEx) begin
        de_ex_ir         <= if_de_ir;
        de_ex_pc         <= if_de_pc;
        de_ex_aluAIn     <= aluAIn;
        de_ex_aluBIn     <= aluBIn;
        de_ex_inst       <= if_de_inst;
        de_ex_iTypeImmed <= iTypeImmed;
    end
end



// ~~~~~~~~~~~~ //
// MEMORY STAGE //
// ~~~~~~~~~~~~ //

wire  [31:0] jalrPc, branchPc, jalPc;      
wire  [31:0] aluRes;
wire   [1:0] pcSel;
logic [31:0] ex_mem_jalrPc=0, ex_mem_branchPc=0, ex_mem_jalPc=0;
logic [31:0] ex_mem_aluRes=0;
logic [31:0] ex_mem_pc=0;
logic  [1:0] ex_mem_pcSel=0;               
instr_t ex_mem_inst;

always_ff @(posedge CLK) begin
    if (!stallMem) begin
        ex_mem_aluRes   <= aluRes;
        ex_mem_pcSel    <= pcSel;
        ex_mem_jalrPc   <= jalrPc;
        ex_mem_branchPc <= branchPc;
        ex_mem_jalPc    <= jalPc;
        ex_mem_inst     <= de_ex_inst;
        ex_mem_pc       <= de_ex_pc;
    end
end
always_comb begin
    IOBUS_OUT <= ex_mem_inst.rs2;
end

// ~~~~~~~~~~~~~~~ //
// WRITEBACK STAGE //
// ~~~~~~~~~~~~~~~ //

wire  [31:0] memData;   
wire  [31:0] dataToRegWrite;
logic [31:0] mem_wb_memData=0;
logic [31:0] mem_wb_pc=0;
logic [31:0] mem_wb_aluRes=0;
instr_t mem_wb_inst;

always_ff @(posedge CLK) begin
    if (!stallWb) begin
        mem_wb_inst    <= ex_mem_inst;
        mem_wb_pc      <= ex_mem_pc;
        mem_wb_aluRes  <= ex_mem_aluRes;
        mem_wb_memData <= memData;
    end
end
always_comb begin
    IOBUS_ADDR <= mem_wb_aluRes;
end

// MODULES

ProgCount prog_count(
    .PC_CLK    (CLK),    
    .PC_RST    (RESET),
    .PC_LD     (~stallIf),  
    .PC_DIN    (nextPc),       // nextPC is provided by MUX (either sequential PC, or branch/jump target)
    .PC_COUNT  (pc)
);
Mult4to1 prog_count_next_mux(
    .In1  (pc + 4),             // Option 1: Current PC + 4
    .In2  (ex_mem_jalrPc),      // Option 2: JALR TARGET calculated during MEM STAGE
    .In3  (ex_mem_branchPc),    // Option 3: BRANCH TARGET calculated during BRANCH STAGE
    .In4  (ex_mem_jalPc),       // Option 4: JAL TARGET calculated during MEM STAGE
    .Sel  (ex_mem_pcSel),       // Determined via PC_SEL calculated during MEM STAGE
    .Out  (nextPc)              
);    

OTTER_mem_byte mem(
    .MEM_CLK     (CLK),
    .MEM_ADDR1   (pc),                        // Current PC -> IR
    .MEM_ADDR2   (ex_mem_aluRes),             // Access memory at the location corresponding to the ALU RESULT during the MEM STAGE 
    .MEM_DIN2    (ex_mem_inst.rs2),           // Save the value from register 2 during the MEM STAGE
    .MEM_WRITE2  (ex_mem_inst.memWrite),      // MEM2 can only be written to if command is STORE
    .MEM_READ1   (~stallIf),                  // An instruction can only be fetched from MEM1 if STALL_FETCH signal is not given
    .MEM_READ2   (ex_mem_inst.memRead2),      // MEM2 can only be read from if command is LOAD
    .IO_IN       (IOBUS_IN),                 
    .MEM_SIZE    (ex_mem_inst.memType[1:0]),
    .MEM_SIGN    (ex_mem_inst.memType[2]),
    .ERR         (),
    .MEM_DOUT1   (ir),                        // Output of MEM1 is the current instruction
    .MEM_DOUT2   (memData),                   // Output of MEM2 is the requested data
    .IO_WR       (IOBUS_WR)
);

OTTER_CU_Decoder decoder(
    .CU_OPCODE     (if_de_ir[6:0]),      // Given from IR @ DECODE STAGE
    .CU_FUNC3      (if_de_ir[14:12]),    // Given from IR @ DECODE STAGE
    .CU_FUNC7      (if_de_ir[31:25]),    // Given from IR @ DECODE STAGE
    .CU_ALU_SRCA   (aluSrcA),            // Used for DECODE STAGE logic
    .CU_ALU_SRCB   (aluSrcB),            // Used for DECODE STAGE logic
    .CU_ALU_FUN    (aluFun),             // Used for EXECUTE STAGE logic
    .CU_RF_WR_SEL  (rfWrSel)             // Used for WRITEBACK STAGE logic
);

OTTER_registerFile reg_file(
    .READ1         (if_de_inst.rfAddr1),        // Given from IR @ DECODE STAGE
    .READ2         (if_de_inst.rfAddr2),        // Given from IR @ DECODE STAGE
    .DEST_REG      (mem_wb_inst.rd),         // Given from inst package @ DECODE STAGE
    .DIN           (dataToRegWrite),         // dataToRegWrite is given from MUX 
    .WRITE_ENABLE  (mem_wb_inst.regWrite),   // The REG FILE can only be updated if the command is neither BRANCH nor LOAD nor STORE
    .OUT1          (rs1),                    // Used for DECODE STAGE logic
    .OUT2          (rs2),                    // Used for DECODE STAGE logic, and saved for MEM STAGE logic
    .CLK           (CLK)
);
Mult4to1 reg_file_data_mux(
    .In1  (mem_wb_pc + 4),             // Option 1: PC from WRITEBACK STAGE + 4
    .In2  (0),                         // Option 2: NONE
    .In3  (mem_wb_memData),            // Option 3: Output of MEM1 from WRITEBACK STAGE
    .In4  (mem_wb_aluRes),             // Option 4: Output of ALU from WRITEBACK STAGE
    .Sel  (ex_mem_inst.rfWrSel),       // Determined by control signal retrieved during MEM STAGE
    .Out  (dataToRegWrite)
);

OTTER_ALU alu(
    .ALU_FUN  (de_ex_inst.aluFun),  // Given from inst package @ EXECUTE STAGE
    .A        (aluAIn),       
    .B        (aluBIn),
    .ALU_OUT  (aluRes)
);
Mult2to1 alu_ain_mux(
    .In1  (rs1),
    .In2  (uTypeImmed),
    .Sel  (aluSrcA),
    .Out  (aluAIn)
);
Mult4to1 alu_bin_mux(
    .In1  (rs2),
    .In2  (iTypeImmed),
    .In3  (sTypeImmed),
    .In4  (if_de_pc),
    .Sel  (aluSrcB),
    .Out  (aluBIn)
);
    
target_gen target_gen(
    .RS1        (rs1),            
    .I_IMMED    (de_ex_iTypeImmed), 
    .PC         (de_ex_pc),      // Is this right? (de_ex_pc)
    .IR         (de_ex_ir),      // Is this right? (de_ex_ir)
    .JALR_PC    (jalrPc),
    .BRANCH_PC  (branchPc),
    .JUMP_PC    (jalPc)
);
immed_gen immed_gen(
    .IR            (if_de_ir),
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

hazard_detector hazard_detector(
    .CLK            (CLK),
    .EX_MEM_RD      (ex_mem_inst.rd),
    .MEM_WB_RD      (mem_wb_inst.rd),
    .DE_EX_RS1      (de_ex_inst.rs1),
    .DE_EX_RS2      (de_ex_inst.rs1),
    .EX_MEM_PC_SEL  (ex_mem_pcSel),
    .STALL_IF       (stallIf),
    .STALL_DE       (stallDe),
    .STALL_EX       (stallEx),
    .STALL_MEM      (stallMem),
    .STALL_WB       (stallWb),
    .INVALID_IF     (invalidIf),
    .INVALID_DE     (invalidDe),
    .INVALID_EX     (invalidEx),
    .INVALID_MEM    (invalidMem),
    .INVALID_WB     (invalidWb)
);


endmodule
