`timescale 1ns / 1ps


module tb();

reg clk, rst;
reg cpu_req_read;
reg cpu_req_valid;
reg [31:0] cpu_req_address;

//reg [63:0] cpu_data_o_;
//reg cache_ready;

reg cpu_req_write; 
reg [63:0] cpu_data_i; 

initial begin
clk = 1;
rst = 1; 
#10;

rst = 0;
#10
cpu_req_write   = 0;
cpu_req_valid   = 1;
cpu_req_read    = 1;
cpu_req_address = 32'b0000_0000_0000_0000_0000_0000_1010_0100; // main memory 164  
                                                               // l1 index 41, l2 index 41 
#130;
cpu_req_valid   = 0; 
#50;
cpu_req_address = 32'b0000_0000_0000_0000_0000_1100_1010_0100; // main memory 3236
cpu_req_valid   = 1;                                           // l1 index 41, l2 index 809
#150;
cpu_req_valid   = 0; 
#40;
cpu_req_valid   = 1;
cpu_req_read    = 1;
cpu_req_address = 32'b0000_0000_0000_0000_0000_0000_1010_0100; // main memory 164 
#150;
cpu_req_valid   = 0;
#20;
cpu_req_valid   = 1;
cpu_req_write   = 1;
cpu_req_read    = 0;
cpu_req_address = 32'b0000_0000_0000_0000_0000_0000_1010_0100; // 164. adrese yaz. 
cpu_data_i = 64'hABCD;
#150;
cpu_data_i = 64'hABDA_ABDA;
#60;
cpu_req_valid   = 0;
#20;
cpu_req_valid   = 1;
#10;
// yazma ardýndan okuma ise güncel veriyi okumak için 1 clk cycle beklemek lazým.  
cpu_req_read    = 1;
cpu_req_address = 32'b0000_0000_0000_0000_0000_0000_1010_0100; 
#120;

 


$finish ;
 
end 

always #5 clk = ~clk;   


top_module dut (.clk(clk) ,.rst(rst) ,.cpu_req_read(cpu_req_read), .cpu_req_write(cpu_req_write) ,.cpu_req_valid(cpu_req_valid) ,.cpu_req_address(cpu_req_address), .cpu_data_i(cpu_data_i));

endmodule
