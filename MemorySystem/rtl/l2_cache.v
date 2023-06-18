
module l2_cache (
		     input clk,
		     input rst, 
		     input [31:0] cpu_addr, 
		     input [63:0] from_l1_data_i,
		     
		     input after_rst_data_is_invalid_flag,
		      
		     input write_from_l1_to_l2,  
		     output [9:0]  l2index,
		     output [19:0] l2tag_valid_dirty,
		     //output reg l1valid,  
		     //output reg l1dirty,    
			
			 input write_from_main_memory_to_l2,
		     input [19:0] update_l2_tag_valid_dirty, 
		     input [63:0] update_l2_data_from_main_memory,
		     
		     output reg l2_cache_req_ready, 
		     output reg data_from_l1_is_loaded,
		     
		    // input read_write_signal, // 0 read, 1 write
		     output wire [63:0] l2data_o

);


// tb de ikinci adresin l1 indexi aynı l2 indexi farklı bu yüzden 
// karşılık gelen l2 indexi 32'bx olmasın diye veri ataması yapıyorum.  


parameter cache_size = 1024;

reg [19:0] l2_tag_array [0:cache_size-1];
reg [63:0] l2data_cache [0:cache_size-1]; 

initial begin
    l2_tag_array[809] <= 32'b0; 
end 


assign l2index  = rst? 10'b0 : cpu_addr[11:2];
//assign current_l2tag_valid_dirty = cpu_addr[31:12]
assign l2tag_valid_dirty    = l2_tag_array[l2index];

assign l2data_o = l2data_cache[l2index];

integer i; 
always@ (posedge clk)  begin

	   
	   if (rst) begin 
                
                data_from_l1_is_loaded <= 1'b0; 
                l2_cache_req_ready <= 1'b0; 
                
        for (i=0; i<1024; i = i +1 ) begin
            
            l2_tag_array[i] <= 20'b0;  
            l2data_cache[i] <= 64'b0;
            
        end
    
	end
	
	
	if (after_rst_data_is_invalid_flag) begin
	    l2_tag_array[l2index] <= update_l2_tag_valid_dirty; 
	end 
	   
	 else if (write_from_main_memory_to_l2) begin
            
            
            l2_tag_array[l2index] <= update_l2_tag_valid_dirty; 
            l2data_cache[l2index] <= update_l2_data_from_main_memory; 
            l2_cache_req_ready <= 1'b1; 
            data_from_l1_is_loaded <= 1'b0; 
            
    end 
    
    else if (write_from_l1_to_l2) begin
        
        l2_tag_array[l2index] <= update_l2_tag_valid_dirty; 
        l2data_cache[l2index] <= update_l2_data_from_main_memory; 
        data_from_l1_is_loaded <= 1'b1; 
         l2_cache_req_ready <= 1'b0; 
        
    end
    
    else begin 
        l2_cache_req_ready <= 1'b0; 
        data_from_l1_is_loaded  = 1'b0; 
    end 
				 
end 



endmodule