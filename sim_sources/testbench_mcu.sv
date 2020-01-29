`timescale 1ns / 1ps

module testbench_mcu();

logic clock;

otter_mcu MCU(
    .CLK(clock)
);

initial begin
    clock = 0;
end

always begin
    #10 clock = ~clock;
end

endmodule
