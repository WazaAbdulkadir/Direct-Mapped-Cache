module main_memory(
			  input  clk,
		      input  rst,
		      input  [31:0] mem_addr,
		      input  [63:0] main_memory_data_from_l2,
		       
		      input mem_req_valid,  
		      input mem_req_rw,
		       
			//  input  read_write_signal, // 0 read, 1 write 	
	     	  
	     	 // memory den istenen veri hazýrsa aktif et. 
		      output reg mem_req_ready, 	  
			  output reg [63:0] main_memory_data_o // çýkan veri 64 bit olmayabilir. s
			);


// cache lerin oluþumuna göre veya LRU yapýsýna göre yazýlacak ve okunacak veri büyüklüðü deðiþebilir. 
parameter size = 65536; 

reg [63:0] main_memory [0:size-1]; 

// cpu addr gelince otomatikmen 64 bit veriyi ver. 
//assign main_memory_data_o = main_memory[mem_addr];

initial begin 
    main_memory[164]   <= 64'b1111;
    main_memory[3236]  <= 64'b0101; 
end 


always@ (posedge clk)  begin
	
	if (rst)begin 
	  mem_req_ready <= 1'b0;
	  //data_o <= x'b0; 
	end 
	
	else if (mem_req_valid && !mem_req_rw)  begin
	       main_memory_data_o <=  main_memory[mem_addr]; 
	       mem_req_ready      <= 1'b1; 
    end
    
    else if (mem_req_valid && mem_req_rw) begin
           main_memory[mem_addr] <= main_memory_data_from_l2;
           mem_req_ready      <= 1'b1; 
           
    end  
    
    else 
        mem_req_ready      <= 1'b0;  
           		 
end  

endmodule 
