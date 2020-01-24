`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 06/07/2018 04:21:54 PM
// Design Name: 
// Module Name: ProgCount
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


module ProgCount(
    input PC_CLK,
    input PC_RST,
    input PC_LD,
    input logic [31:0] PC_DIN,
    output logic [31:0] PC_COUNT=0
    );
    
    // on the positive edge of a clock
    always_ff @(posedge PC_CLK)
    begin
        // if the reset signal is sent
        if (PC_RST == 1'b1)
            // set PC_COUNT to 0
            PC_COUNT <= '0;
        // if load is set
        else if (PC_LD == 1'b1)
            // set PC_COUNT to new value
            PC_COUNT <= PC_DIN;
    end
    
endmodule
