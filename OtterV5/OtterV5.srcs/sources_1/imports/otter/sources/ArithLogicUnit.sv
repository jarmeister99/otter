`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes 
// 
// Create Date: 06/07/2018 05:03:50 PM
// Design Name: 
// Module Name: ArithLogicUnit
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

module OTTER_ALU(ALU_FUN, A, B, ALU_OUT);
        input [3:0] ALU_FUN;  //func7[5],func3
        input [31:0] A,B;
        output logic [31:0] ALU_OUT; 
       
        always_comb
        begin //reevaluate If these change
            case(ALU_FUN)
                0:  ALU_OUT = A + B;     //add
                8:  ALU_OUT = A - B;     //sub
                6:  ALU_OUT = A | B;     //or
                7:  ALU_OUT = A & B;     //and
                4:  ALU_OUT = A ^ B;     //xor
                5:  ALU_OUT =  A >> B[4:0];    //srl
                1:  ALU_OUT =  A << B[4:0];    //sll
               13:  ALU_OUT =  $signed(A) >>> B[4:0];    //sra
                2:  ALU_OUT = $signed(A) < $signed(B) ? 1: 0;       //slt
                3:  ALU_OUT = A < B ? 1: 0;      //sltu
                9:  ALU_OUT = A; //copy op1 (lui)
                10: ALU_OUT = A * B;
                default: ALU_OUT = 0; 
            endcase
        end
    endmodule
   
  