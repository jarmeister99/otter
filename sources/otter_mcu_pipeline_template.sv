`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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
    logic [31:0] ir, pc, rs2;
} instr_t;

module OTTER_MCU(input  CLK,
                 input  INTR,
                 input  RESET,
                 input  [31:0] IOBUS_IN,
                 output [31:0] IOBUS_OUT,
                 output [31:0] IOBUS_ADDR,
                 output logic IOBUS_WR 
);   
	
    wire [31:0]  pc,                               // Tied to the output of the PROG_COUNTER
                 nextPc,                           // Tied to the output of the PC 4-1 MUX
                 jalrPc, branchPc, jalPc,          // Tied to the outputs of the TARGET_GENERATOR (uses EXECUTE state)
                 A,B,                              // Tied to the asynchronous outputs of the REG_FILE
                 ir,                               // Tied to the output of the PROGRAM_MEMORY 
                 iTypeImmed,sTypeImmed,uTypeImmed, // Tied to the outputs of the IMMEDIATE_GENERATOR (uses DECODE state)
                 aluAin,aluBin,                    // Tied to the outputs of the ALU 2-1 and 4-2 MUX's (uses DECODE state)
                 aluResult,                        // Tied to the output of the ALU (uses EXECUTE state)
                 rfIn,                             // Tied to the output of the REG_FILE 4-1 MUX
                 memData;                          // Tied to the output of the MEMORY_FILE (uses MEMORY state)                             
               
    logic [1:0]  pcSel;
    logic        brLt, brEq, brLtu,
                 stallPc, stallIf, stallDe, stallEx=0, stallMem=0, stallWb=0,
                 ifDeInvalid=0, deExInvalid=0, exMemInvalid=0, memWbInvalid=0;
     
//==== Instruction Fetch ===========================================

     // STATE VARIABLES
     logic [31:0] if_de_pc;
     
     // COMB VARIABLES
     wire pcWrite, memRead1;
     
     
     // ASSIGN STATE VARIABLES
     always_ff @(posedge CLK) begin
            if(!stallIf) begin
                if_de_pc <= pc;
            end
     end
     
     // ASSIGN COMB. VARIABLES
     assign pcWrite = !stallIf;
     assign memRead1 = !stallIf;
     


     
//==== Instruction Decode ===========================================
    
    // STATE VARIABLES
    logic [31:0] de_ex_iTypeImmed;
    logic [31:0] de_ex_aluAin, de_ex_aluBin;
    instr_t      de_ex_inst;
    
    // COMB. VARIABLES
    opcode_t   opcode;
    wire [1:0] opBSel; // given by decoder 
    wire       opASel; // ...
    
    
    // ASSIGN STATE VARIABLES
    always_ff @(posedge CLK) begin
        if (!stallDe) begin

            de_ex_aluAin <= aluAin;
            de_ex_aluBin <= aluBin;
            de_ex_inst.rs2 <= B;
            de_ex_inst.rfAddr1 <= ir[19:15];
            de_ex_inst.rfAddr2 <= ir[24:20];
            de_ex_inst.rd <= ir[11:7];
            de_ex_inst.pc <= pc;
            de_ex_inst.opcode <= opcode;
            de_ex_inst.ir <= ir;
            de_ex_inst.memRead2 <= opcode==LOAD;                // Set memory reading control signal if appropriate
            de_ex_inst.memWrite <= opcode==STORE;               // Set memory writing control signal if appropriate
            de_ex_inst.regWrite <= opcode != BRANCH &&          // Set register writing control signal if appropriate
                                   opcode != LOAD   &&
                                   opcode != STORE;                          
            de_ex_inst.rs1Used <= A                    != 0      
                                  && de_ex_inst.opcode != LUI
                                  && de_ex_inst.opcode != AUIPC
                                  && de_ex_inst.opcode != JAL;    
                                        
            de_ex_iTypeImmed <= iTypeImmed;
                  
        end
    end         
    
    // ASSIGN COMB. VARIABLES
    assign opcode = opcode_t'(ir[6:0]);
    
    // Immediate Generator
    assign sTypeImmed = {{20{de_ex_inst.ir[31]}},de_ex_inst.ir[31:25],de_ex_inst.ir[11:7]};
    assign iTypeImmed = {{20{de_ex_inst.ir[31]}},de_ex_inst.ir[31:20]};
    assign uTypeImmed = {de_ex_inst.ir[31:12],{12{1'b0}}};
    
    
    //===== HAZARD DETECTION =================================
    //stall on load-use
    //assign stall_if = 
	
    logic branch_taken;
    assign branch_taken = !deExInvalid && (pcSel != 0);    

    always_ff @ (posedge CLK) begin
        if(RESET) begin
            ifDeInvalid<=1;
            deExInvalid<=1;
            exMemInvalid<=1;
            memWbInvalid<=1;
        end
        else begin         
            if(!stallIf) ifDeInvalid <=branch_taken;
            if(!stallDe) deExInvalid <= ifDeInvalid | branch_taken;
            else if (!stallEx) deExInvalid <= 1;
           
            if(!stallEx) exMemInvalid <= deExInvalid;
            // If we are not supposed to stall MEM, MEM_WB is invalid if EX_MEM is invalid
            if(!stallMem) memWbInvalid <= exMemInvalid;
        end
    end
    
//==== Execute ======================================================

     // STATE VARIABLES
     logic [31:0] ex_mem_iTypeImmed;
     logic        ex_mem_aluRes = 0;
     instr_t      ex_mem_inst;

     // COMB. VARIABLES
     logic [31:0] opAForwarded, opBForwarded; // assigned via forwarding logic
     
     // ASSIGN STATE VARIABLES
     always_ff @(posedge CLK) begin
         if(!stallEx) begin
             ex_mem_aluRes <= aluResult;
             ex_mem_inst <= de_ex_inst;
             ex_mem_iTypeImmed <= de_ex_iTypeImmed;
         end
     end
         
     // ASSIGN COMB. VARIABLES
     always_comb begin
        if (!opAForwarded) begin
            opAForwarded <= aluAin;
        end
        if (!opBForwarded) begin
            opBForwarded <= aluBin;
        end
     end
     
     //Branch Condition Generator
     always_comb begin
         brLt=0; brEq=0; brLtu=0;
         if($signed(de_ex_aluAin) < $signed(de_ex_aluBin)) brLt=1;
         if(de_ex_aluAin==de_ex_aluBin) brEq=1;
         if(de_ex_aluAin<de_ex_aluBin) brLtu=1;
     end

     
//==== Memory ======================================================
     
     
    // STATE VARIABLES
    logic [31:0] mem_wb_data;
    logic [31:0] mem_wb_aluRes;
    instr_t mem_wb_inst;
    
    
    // COMB. VARIABLES
    
    
    
    // ASSIGN STATE VARIABLES
    always_ff @(posedge CLK) begin
        if(!stallMem) begin
            mem_wb_data <= memData;
            mem_wb_inst <= ex_mem_inst;
            mem_wb_aluRes <= ex_mem_aluRes;
        end
    end
    
    // ASSIGN COMB. VARIABLES
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_inst.rs2;
    
    // Target Generator
    assign jalrPc = de_ex_iTypeImmed + A;
    assign branchPc = de_ex_inst.pc + {{20{de_ex_inst.ir[31]}},de_ex_inst.ir[7],de_ex_inst.ir[30:25],de_ex_inst.ir[11:8],1'b0};   //byte aligned addresses
    assign jalPc = de_ex_inst.pc + {{12{de_ex_inst.ir[31]}}, de_ex_inst.ir[19:12], de_ex_inst.ir[20],de_ex_inst.ir[30:21],1'b0};
     
     
     
//==== Write Back ==================================================

    logic [31:0] wd;




//==== Modules ===============
    
     OTTER_registerFile reg_file(
        .Read1(ir[19:15]), 
        .Read2(ir[24:20]), 
        .WriteReg(mem_wb_inst.rd), 
        .WriteData(wd),
        .RegWrite(mem_wb_inst.regWrite), .Data1(A), .Data2(B), .clock(CLK));
     Mult4to1 reg_file_wd_mux(
        .In1(mem_wb_inst.pc + 4),
        .In2(0), .In3(mem_wb_data),
        .In4(mem_wb_aluRes),
        .Sel(mem_wb_inst.rfWrSel),
        .Out(rfIn));
        
     OTTER_mem_byte memory(
        .MEM_CLK(CLK),
        .MEM_ADDR1(pc),
        .MEM_ADDR2(ex_mem_aluRes),
        .MEM_DIN2(ex_mem_inst.rs2),
        .MEM_WRITE2(ex_mem_inst.memWrite),
        .MEM_READ1(memRead1),
        .MEM_READ2(memRead2),
        .ERR(),
        .MEM_DOUT1(ir),
        .MEM_DOUT2(memData),
        .IO_IN(IOBUS_IN),
        .IO_WR(IOBUS_WR),
        .MEM_SIZE(ex_mem_inst.memType[1:0]),
        .MEM_SIGN(ex_mem_inst.memType[2]));
       
     OTTER_ALU alu(
        .ALU_fun(de_ex_inst.aluFun),
        .A(opAForwarded),
        .B(opBForwarded),
        .ALUOut(aluResult)); 
     Mult2to1 alu_a_mux(
        .In1(A),
        .In2(uTypeImmed),
        .Sel(opASel),
        .Out(aluAin));
     Mult4to1 alu_b_mux(.In1(B),
        .In2(iTypeImmed),
        .In3(sTypeImmed),
        .In4(ex_mem_inst.pc),
        .Sel(opBSel),
        .Out(aluBin));
     
     ProgCount prog_count(
        .PC_CLK(CLK),
        .PC_RST(RESET),
        .PC_LD(pcWrite),
        .PC_DIN(nextPc),
        .PC_COUNT(pc));
     Mult4to1 prog_count_next_mux(
        .In1(ex_mem_inst.pc + 4),
        .In2(jalrPc),
        .In3(branchPc),
        .In4(jalPc),
        .Sel(pcSel),
        .Out(nextPc));
     
     OTTER_CU_Decoder decoder(
        .CU_OPCODE(de_ex_inst.opcode),
        .CU_FUNC3(de_ex_inst.ir[14:12]),
        .CU_FUNC7(de_ex_inst.ir[31:25]),
        .CU_BR_EQ(brEq), 
        .CU_BR_LT(brLt), 
        .CU_BR_LTU(brLtu), 
        .CU_ALU_SRCA(opASel), 
        .CU_ALU_SRCB(opBSel),
        .CU_ALU_FUN(de_ex_inst.aluFun), 
        .CU_RF_WR_SEL(de_ex_inst.rfWrSel), 
        .CU_PCSOURCE(pcSel));
     
        
       
    


 
 


 
       
 //==== Forwarding Logic ===========================================
 
 
 //==== Modules ===========================================
     




          
       
            
endmodule
