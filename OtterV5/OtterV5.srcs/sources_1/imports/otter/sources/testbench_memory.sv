`timescale 1ns / 1ps

module testbench_memory();

logic [31:0] MEM_ADDR1;     //Instruction Memory Port
logic [31:0] MEM_ADDR2;     //Data Memory Port
logic MEM_CLK;
logic [31:0] MEM_DIN2;
logic MEM_WRITE2;
logic MEM_READ1;
logic MEM_READ2;
logic [31:0] IO_IN;
logic ERR;
logic [1:0] MEM_SIZE;
logic MEM_SIGN;
logic [31:0] MEM_DOUT1;
logic [31:0] MEM_DOUT2;
logic IO_WR;

initial begin
MEM_CLK = 1'B0;
MEM_ADDR1 = 32'B00000000000000000000000000000000;
end

OTTER_mem_byte uut(.MEM_ADDR1(MEM_ADDR1), .MEM_ADDR2(MEM_ADDR2), .MEM_CLK(MEM_CLK), .MEM_DIN2(MEM_DIN2), 
        .MEM_WRITE2(MEM_WRITE2), .MEM_READ1(MEM_READ1), .MEM_READ2(MEM_READ2), .IO_IN(IO_IN), .ERR(ERR), 
        .MEM_SIZE(MEM_SIZE), .MEM_SIGN(MEM_SIGN), .MEM_DOUT1(MEM_DOUT1), .MEM_DOUT2(MEM_DOUT2), .IO_WR(IO_WR)); 
   
always
#10 MEM_CLK = ~MEM_CLK;
       
endmodule 

