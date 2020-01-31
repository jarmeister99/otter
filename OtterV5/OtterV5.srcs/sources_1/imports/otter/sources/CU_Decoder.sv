`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 01/27/2019 09:22:55 AM
// Design Name: 
// Module Name: CU_Decoder
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
//`include "opcodes.svh"

module OTTER_CU_Decoder(
    input [6:0] CU_OPCODE,
    input [2:0] CU_FUNC3,
    input [6:0] CU_FUNC7,
    output logic CU_ALU_SRCA,
    output logic [1:0] CU_ALU_SRCB,
    output logic [3:0] CU_ALU_FUN,
    output logic [1:0] CU_RF_WR_SEL
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
        
       
        opcode_t OPCODE;
        assign OPCODE = opcode_t'(CU_OPCODE);
        
        logic brn_cond;
        always_comb
            case(CU_OPCODE)
                OP_IMM:     CU_ALU_FUN = (CU_FUNC3==3'b101)?{CU_FUNC7[5],CU_FUNC3}:{1'b0,CU_FUNC3};
                LUI,SYSTEM: CU_ALU_FUN = 4'b1001;
                OP:         CU_ALU_FUN = {CU_FUNC7[5],CU_FUNC3};
                default:    CU_ALU_FUN = 4'b0;
            endcase        
            
         always_comb  begin
            case(CU_OPCODE)
                JAL:     CU_RF_WR_SEL=0;
                JALR:    CU_RF_WR_SEL=0;
                LOAD:    CU_RF_WR_SEL=2;
                SYSTEM:  CU_RF_WR_SEL=1;
                default: CU_RF_WR_SEL=3; 
            endcase 
          end   
          
          
         always_comb begin
             case(CU_OPCODE)
                 STORE:  CU_ALU_SRCB=2;  //S-type
                 LOAD:   CU_ALU_SRCB=1;  //I-type
                 JAL:    CU_ALU_SRCB=1;  //I-type
                 OP_IMM: CU_ALU_SRCB=1;  //I-type
                 AUIPC:  CU_ALU_SRCB=3;  // U-type (special) LUI does not use B
                 default:CU_ALU_SRCB=0;  //R-type    //OP  BRANCH-does not use
             endcase
         end
                     
       assign CU_ALU_SRCA = (CU_OPCODE==LUI || CU_OPCODE==AUIPC) ? 1 : 0;  

endmodule
