module l1_cache(
		     input clk,
		     input rst, 
		    
		     // cpu adresi cache lere doğrudan bağlamak sıkıntı oluştureabilir.
		     // aytı bir reg olarak atabilirim. 
		     input [31:0] cpu_addr,  // adres yerine tag ve index girdi olarak verilebilir. 
		     input [63:0] data_i,
		     
		     input after_rst_data_is_invalid_flag,
		    
		     output [7:0]  l1index,
		     output [23:0] l1tag_valid_dirty,
		     //output reg l1valid,  
		     //output reg l1dirty,    
			
			 input write_from_l2_to_l1,
		     input [23:0] update_l1_tag_valid_dirty, 
		     input [63:0] update_l1_data_from_l2,
		     
		     input write_cpu_data_to_l1_cache,
		     
		     
		     
		   // input read_write_signal, // 0 read, 1 write
		     output wire [63:0] l1data_o

);

parameter cache_size = 256;

reg [63:0] l1data_cache [0:cache_size-1]; 

// tag_array'in 24. biti valid bit, 23.biti dirty bit. Kalan 22 bit tag.  
reg [23:0] l1_tag_array [0:cache_size -1]; 

//wire [21:0] l1tag; 
//reg l1_valid_bit [0:cache_size-1] ;
//reg l1_dirty_bit [0:cache_size-1] ; 

assign l1index  = rst? 8'b0 :cpu_addr[9:2];
//assign l1tag   =  cpu_addr[31:10];
assign l1tag_valid_dirty    = l1_tag_array[l1index];
assign l1data_o = l1data_cache[l1index];
  
    
  //  genvar i; 
    
integer i;    
    
always@ (posedge clk)  begin
	if (rst) begin 
    
        for (i=0; i<256; i = i +1 ) begin
            
            l1_tag_array[i] <= 24'b0;  
            l1data_cache[i] <= 64'b0;
        end
    
	end

    else begin 
        
        if (after_rst_data_is_invalid_flag) begin
            l1_tag_array[l1index] <= update_l1_tag_valid_dirty; 
        end 
        
         else if (write_from_l2_to_l1 ) begin
            
            
            l1_tag_array[l1index] <= update_l1_tag_valid_dirty; 
            l1data_cache[l1index] <= update_l1_data_from_l2; 
            
        end 
        
        else if (write_cpu_data_to_l1_cache) begin
                
            l1_tag_array[l1index] <= update_l1_tag_valid_dirty; 
            l1data_cache[l1index] <= data_i;  
        
        end 
        
    
end 

	// tag_array ve l1data_cache verilerinin güncellenmesi 
//	else if ((l1tag == l1_tag_array [l1index][21:0]) ) begin
	
//	      if (read_write_signal== 1)  
//	           l1data_cache[l1index] <= data_i; 
//		 else if(read_write_signal== 0 && l1_tag_array[l1index][23])
//	           //l1data_o = l1data_cache[l1index];
//	end 
	
	// bu kod uygun bir yere yerleştirilecek. 
	//l1valid  = l1_valid_bit[l1index];
	
end 



endmodule