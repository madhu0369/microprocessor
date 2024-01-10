//Testbench of Microprocessor

`timescale 1ns / 1ps
module microprocessor_tb();

reg clk = 0,sys_rst = 0;
reg [15:0] din = 0;
wire [15:0] dout;


microprocessor dut(clk, sys_rst, din, dout); //Instantiation of the module microprocessor

//CLOCK SETUP
always #5 clk = ~clk;
 
initial begin
sys_rst = 1'b1;
repeat(5) @(posedge clk);
sys_rst = 1'b0;
#2000;
$stop;
end
endmodule