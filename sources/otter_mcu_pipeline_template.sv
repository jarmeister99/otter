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
    logic [4:0] rf_addr1;
    logic [4:0] rf_addr2;
    logic [4:0] rd;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc, rs1, rs2;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
	wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] ir;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
    
    logic [31:0] rs1_forwarded;
    logic [31:0] rs2_forwarded;

    logic br_lt,br_eq,br_ltu;
              
     logic stall_pc;
     logic stall_if;
     logic stall_de;
     //The following stages are not stalled in our first piplined otter  (no interrupts, exceptions, memory delays)
     logic stall_ex=0;
     logic stall_mem=0;
     logic stall_wb=0;
     
     logic if_de_invalid=0;
     logic de_ex_invalid=0;
     logic ex_mem_invalid=0;
     logic mem_wb_invalid=0;
     
//==== Instruction Fetch ===========================================

     // Save the PC on each clock cycle (if appropriate) 
     logic [31:0] if_de_pc;
     
     // On a clock edge, save PC to IF_DE register if this stage is not supposed to stall
     always_ff @(posedge CLK) begin
            if(!stall_if) begin
                if_de_pc <= pc;
            end
     end
     
     // Assert pcWrite control signal if we are not supposed to stall (used to increment PC)
     assign pcWrite = !stall_if;
     
     // Assert memRead1 signal if we are not supposed to stall (used to fetch instruction)
     assign memRead1 = !stall_if;
     


     
//==== Instruction Decode ===========================================
    
    // Make structure that holds state
    instr_t de_ex_inst;
    
    // Declare an opcode type logic
    opcode_t OPCODE;
    
    // Cast opcode to an opcode type logic
    assign OPCODE = opcode_t'(ir[6:0]);     
    
    assign S_immed = {{20{de_ex_inst.ir[31]}},de_ex_inst.ir[31:25],de_ex_inst.ir[11:7]};
    assign I_immed = {{20{de_ex_inst.ir[31]}},de_ex_inst.ir[31:20]};
    assign U_immed = {de_ex_inst.ir[31:12],{12{1'b0}}};
        
    always_ff @(posedge CLK) begin
        if (!stall_de) begin

            
            // SAVE addr1, addr2, rd to DE_EX register
            assign de_ex_inst.rf_addr1=ir[19:15];
            assign de_ex_inst.rf_addr2=ir[24:20];
            assign de_ex_inst.rd=ir[11:7];
            
            // SAVE current PC to DE_EX register
            assign de_ex_inst.pc=pc;
            
            // SAVE OPCODE to DE_EX register
            assign de_ex_inst.opcode=OPCODE;
            
            // SAVE whether rs1 should be used to DE_EX register
            assign de_ex_inst.rs1_used=    de_ex_inst.rs1 != 0
                                        && de_ex_inst.opcode != LUI
                                        && de_ex_inst.opcode != AUIPC
                                        && de_ex_inst.opcode != JAL;          
        end
    end                                                             
    
    //===== HAZARD DETECTION =================================
    //stall on load-use
    //assign stall_if = 
	
	
	
    //For instruction that is branch/jump, if changes the PC,  
    logic branch_taken;
    
    // Set a flag if the branch is supposed to be taken
    assign branch_taken = !de_ex_invalid && (pc_sel != 0);    

    always_ff @ (posedge CLK) begin
        // If a reset signal is given, mark all stages as invalid
        if(RESET) begin
            if_de_invalid<=1;
            de_ex_invalid<=1;
            ex_mem_invalid<=1;
            mem_wb_invalid<=1;
        end
        else begin         
            // If we are not supposed to stall IF, IF_DE stage is marked as invalid if we branched
            if(!stall_if) if_de_invalid <=branch_taken;
            
            // If we are not supposed to stall DE, DE_EX stage is marked as invalid if DE is invalid or we branched
            if(!stall_de) de_ex_invalid <= if_de_invalid | branch_taken;
            // Else, if we are not supposed to stall EX, DE_EX is marked as invalid
            else if (!stall_ex) de_ex_invalid <= 1;
            
            // If we are not supposed to stall EX, EX_MEM is invalid if DE_EX is invalid
            if(!stall_ex) ex_mem_invalid <= de_ex_invalid;
            // If we are not supposed to stall MEM, MEM_WB is invalid if EX_MEM is invalid
            if(!stall_mem) mem_wb_invalid <= ex_mem_invalid;
        end
    end
    
//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic ex_mem_aluRes = 0;
     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;

    
     always_ff @(posedge CLK) begin
     
        // If this stage is not supposed to stall
        if(!stall_ex) begin
        
            // SAVE the result of the ALU
            ex_mem_aluRes <= aluResult;
            
            // SAVE state from previous register
            ex_mem_inst <= de_ex_inst;
            
        end
     end
    
     




//==== Memory ======================================================
     
    logic [31:0] mem_wb_data;
    logic [31:0] mem_wb_aluRes;
    instr_t mem_wb_inst;
    
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    always_ff @(posedge CLK) begin
        if(!stall_mem) begin
            // On clock edge... 
            // SAVE data from Memory Module DOUT2
            mem_wb_data <= mem_data;
            
            // SAVE state from previous register
            mem_wb_inst <= ex_mem_inst;
            
            // SAVE aluRes from previous register
            mem_wb_aluRes <= ex_mem_aluRes;
        end
    end
    
     // Create program memory
     
     
     
//==== Write Back ==================================================

    logic wd[31:0];
    




    



//==== Modules ===============
    
     OTTER_registerFile reg_file (.Read1(ir[19:15]), .Read2(ir[24:20]), .writeData(wd), .WriteReg(mem_wb_inst.regWrite),
        .Data1(de_ex_inst.rs1), .Data2(de_ex_inst.rs2), .clock(clk));
        
     Mult4to1 reg_file_wd_mux (.In1(mem_wb_inst.pc + 4), .In2(0), .In3(mem_wb_data), .In4(mem_wb_aluRes), .Sel(mem_wb_rf_wr_sel), .Out(wd));
        
     OTTER_mem_byte memory (.MEM_CLK(clk), .MEM_ADDR1(pc), .MEM_ADDR2(ex_mem_aluRes), .MEM_DIN2(ex_mem_inst.rs2),
        .MEM_WRITE2(ex_mem_inst.memWrite), .MEM_READ1(memRead1), .ERR(), .MEM_DOUT1(ir), .MEM_DOUT2(mem_data), .IO_IN(IOBUS_IN),
        .IO_WR(IOBUS_WR), .MEM_SIZE(ex_mem.mem_type[1:0]), .MEM_SIGN(ex_mem.mem_type[2]));
       
     OTTER_ALU alu (de_ex_inst.alu_fun, opA_forwarded, opB_forwarded, aluResult); 
     Mult2to1 alu_a_mux (.In1(ex_mem_inst.rs1), .In2(U_immed), .Sel(opA_sel), .Out(A));
     Mult4to1 alu_b_mux (.In1(ex_mem_inst.rs2), .In2(I_immed), .In3(S_immed), .In4(ex_mem_inst.pc), .Sel(opB_sel), .Out(B));
     
     ProgCount prog_count (.PC_CLK(clk), .PC_RST(RESET), .PC_LD(pcWrite), .PC_DIN(pc_value), .PC_COUNT(pc));
     Mult4to1 prog_count_next_mux (.In1(ex_mem_inst.pc + 4), .In2(jalr_pc), .In3(branch_pc), .In4(jal_pc), .Sel(pc_sel), .Out(pc_value));
     
     OTTER_CU_Decoder decoder (.CU_OPCODE(de_ex_inst.opcode), .CU_FUNC3(de_ex_inst.ir[14:12]), .CU_FUNC7(de_ex_inst.ir[31:25]),
        .CU_BR_EQ(-1), .CU_BR_LT(-1), .CU_BR_LTU(-1), .INT_TAKEN(0), .CU_ALU_SRCA(opA_sel), .CU_ALU_SRCB(opB_sel),
        .CU_ALU_FUN(de_ex_inst.alu_fun), .CU_RF_WR_SEL(de_ex_inst.rf_wr_sel), .CU_PC_SOURCE(pc_sel))
     
        
       
    


 
 


 
       
 //==== Forwarding Logic ===========================================
 
 
 //==== Modules ===========================================
     




          
       
            
endmodule
