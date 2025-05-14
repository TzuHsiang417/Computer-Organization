`timescale 1ns / 1ps

module CPU_A_tb;

reg clk;
wire [31:0] PC;

CPU_A test( .clk(clk) , .PC(PC));

initial 
    begin
        clk = 1'b0;
        #20;
        forever #20 clk = ~clk;
    end

initial 
    begin
        #880; $finish;
    end
    
endmodule
