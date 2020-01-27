`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/05/2019 12:17:57 AM
// Design Name: 
// Module Name: registerFile
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


module OTTER_registerFile(READ1,READ2,DEST_REG,DIN,WRITE_ENABLE,OUT1,OUT2,CLK);
    input [4:0] READ1,READ2,DEST_REG; //the register numbers to read or write
    input [31:0] DIN; //data to write
    input WRITE_ENABLE; //the write control
    input CLK;  // the CLK to trigger write
    output logic [31:0] OUT1, OUT2; // the register values read
    logic [31:0] RF [31:0]; //32 registers each 32 bits long
    
    //assign OUT1 = RF[READ1];
    //assign OUT2 = RF[READ2];
    always_comb
        if(READ1==0) OUT1 =0;
        else OUT1 = RF[READ1];
    always_comb
        if(READ2==0) OUT2 =0;
        else OUT2 = RF[READ2];
    
    always@(negedge CLK) begin // write the register with the new value if WRITE_ENABLE is high
        if(WRITE_ENABLE && DEST_REG!=0) RF[DEST_REG] <= DIN;
        
    end
 endmodule

