`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/26/2020 01:43:42 PM
// Design Name: 
// Module Name: branch_cond_gen
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

module branch_cond_gen(
    input  [31:0] A,
    input  [31:0] B,
    input  [6:0]  OPCODE,
    input  [2:0]  FUNC3,
    output logic  [1:0] PC_SEL
);

logic brLt;
logic brEq;
logic brLtu;

always_comb begin
    brLt=0; brEq=0; brLtu=0;
    if($signed(A) < $signed(B)) brLt=1;
    if(A==B)                    brEq=1;
    if(A<B)                     brLtu=1;    
end

logic brnCond;

always_comb
 begin
    case(FUNC3)
        3'b000: brnCond = brEq;     //BEQ 
        3'b001: brnCond = ~brEq;    //BNE
        3'b100: brnCond = brLt;     //BLT
        3'b101: brnCond = ~brLt;    //BGE
        3'b110: brnCond = brLtu;    //BLTU
        3'b111: brnCond = ~brLtu;   //BGEU
        default: brnCond =0;
   endcase
end

always_comb begin
    case(OPCODE)
        7'b1101111: PC_SEL=2'b11;
        7'b1100111: PC_SEL=2'b01;
        7'b1100011: PC_SEL=(brnCond)?2'b10:2'b00;
        default:    PC_SEL=2'b00; 
    endcase  
end

endmodule
