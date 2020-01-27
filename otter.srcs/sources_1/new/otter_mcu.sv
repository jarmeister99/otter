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
    logic [31:0] rs2;
} instr_t;

module otter_mcu(
    input CLK,
    input RESET,
    input  [31:0] IOBUS_IN,
    input  [31:0] IOBUS_ADDR,
    output logic [31:0] IOBUS_OUT,
    output logic IOBUS_WR
);

// PIPELINE CONTROLS
logic stallIf=0, stallDe=0, stallEx=0, stallMem=0, stallWb=0; 

// ~~~~~~~~~~~ //
// FETCH STAGE //
// ~~~~~~~~~~~ //

wire  [31:0] pc, nextPc;

// ~~~~~~~~~~~~ //
// DECODE STAGE //
// ~~~~~~~~~~~~ //

wire  [31:0] sTypeImmed, iTypeImmed, uTypeImmed;
wire  [31:0] ir;         // May need to be stage reg?
wire   [3:0] aluFun;
wire   [1:0] aluSrcB;
wire         aluSrcA;
logic [31:0] if_de_pc=0;
logic [31:0] if_de_ir=0;

always_ff @(posedge CLK) begin
    if (!stallDe) begin
        if_de_pc <= pc;
        if_de_ir <= ir;
    end
end

// ~~~~~~~~~~~~~ //
// EXECUTE STAGE //
// ~~~~~~~~~~~~~ //

wire  [31:0] aluAIn, aluBIn;
wire  [31:0] rs1, rs2;
logic [31:0] de_ex_ir=0;
logic [31:0] de_ex_pc=0;
logic [31:0] de_ex_aluAIn=0, de_ex_aluBIn=0;
logic  [1:0] rfWrSel;
logic        brLt, brEq, brLtu;
opcode_t opcode;
instr_t de_ex_inst;

always_ff @(posedge CLK) begin
    if (!stallEx) begin
        de_ex_ir     <= if_de_ir;
        de_ex_pc     <= if_de_pc;
        de_ex_aluAIn <= aluAIn;
        de_ex_aluBIn <= aluBIn;
    end
end

assign opcode              = opcode_t'(de_ex_ir[6:0]);
assign de_ex_inst.opcode   = opcode;
assign de_ex_inst.aluFun   = aluFun;
assign de_ex_inst.rfWrSel  = rfWrSel;
assign de_ex_inst.func3    = de_ex_ir[14:12];
assign de_ex_inst.rd       = de_ex_ir[11:7];
assign de_ex_inst.rs2      = rs2;
assign de_ex_inst.memType  = de_ex_ir[14:12];
assign de_ex_inst.memWrite = de_ex_inst.opcode==STORE;
assign de_ex_inst.memRead2 = de_ex_inst.opcode==LOAD;
assign de_ex_inst.regWrite = de_ex_inst.opcode != BRANCH     &&
                                 de_ex_inst.opcode != LOAD   &&
                                 de_ex_inst.opcode != STORE;    // Maybe move to next stage?

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
        IOBUS_OUT       <= ex_mem_inst.rs2;
    end
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

// MODULES

ProgCount prog_count(
    .PC_CLK    (CLK),    
    .PC_RST    (RESET),
    .PC_LD     (~stallIf),  
    .PC_DIN    (nextPc),   
    .PC_COUNT  (pc)
);
Mult4to1 prog_count_next_mux(
    .In1  (pc + 4),             
    .In2  (ex_mem_jalrPc),             
    .In3  (ex_mem_branchPc),          
    .In4  (ex_mem_jalPc),            
    .Sel  (ex_mem_pcSel),      
    .Out  (nextPc)
);    

OTTER_mem_byte mem(
    .MEM_CLK     (CLK),
    .MEM_ADDR1   (pc),
    .MEM_ADDR2   (ex_mem_aluRes),               
    .MEM_DIN2    (ex_mem_inst.rs2),
    .MEM_WRITE2  (ex_mem_inst.memWrite),     // hook this up
    .MEM_READ1   (~stallIf),
    .MEM_READ2   (ex_mem_inst.memRead2),     // hook this up
    .IO_IN       (IOBUS_IN),
    .MEM_SIZE    (ex_mem_inst.memType[2:1]),
    .MEM_SIGN    (ex_mem_inst.memType[0]),
    .ERR         (),
    .MEM_DOUT1   (ir),
    .MEM_DOUT2   (memData),
    .IO_WR       (IO_WR)
);

OTTER_CU_Decoder decoder(
    .CU_OPCODE     (if_de_ir[6:0]),
    .CU_FUNC3      (if_de_ir[14:12]),
    .CU_FUNC7      (if_de_ir[31:25]),
    .CU_ALU_SRCA   (aluSrcA),
    .CU_ALU_SRCB   (aluSrcB),
    .CU_ALU_FUN    (aluFun),           
    .CU_RF_WR_SEL  (rfWrSel)            
);

OTTER_registerFile reg_file(
    .READ1         (if_de_ir[19:15]),
    .READ2         (if_de_ir[24:20]),
    .DEST_REG      (mem_wb_inst.rd),
    .DIN           (dataToRegWrite),
    .WRITE_ENABLE  (mem_wb_inst.regWrite),    // hook this up 
    .OUT1          (rs1),
    .OUT2          (rs2),
    .CLK           (CLK)
);
Mult4to1 reg_file_data_mux(
    .In1  (mem_wb_pc + 4),             
    .In2  (0),             
    .In3  (mem_wb_data),          
    .In4  (mem_wb_aluRes),            
    .Sel  (ex_mem_inst.rfWrSel),      
    .Out  (dataToRegWrite)
);

OTTER_ALU alu(
    .ALU_FUN  (de_ex_inst.aluFun),
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
    .I_IMMED    (iTypeImmed), 
    .PC         (de_ex_pc),      // Is this right?
    .IR         (de_ex_ir),      // Is this right?
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


endmodule
