`timescale 1ns / 1ps


module top_module( 
                // signals and arrays coming from CPU or Bus 
                   input rst,
                   input clk,
                   input cpu_req_read,
                   input cpu_req_write, 
                   input cpu_req_valid,
                   input [31:0] cpu_req_address,
                   input [63:0] cpu_data_i,
                   
            // following outputs goes to cpu or bus 
                   output wire [63:0] cpu_data_o_,
               // cache can take request    
                   output wire cache_ready
                   
                    );


//wire rst_flag; 
wire after_rst_data_is_invalid_flag; 
wire data_from_l1_is_loaded;
// signal and arrays mostly related with l1 cache
wire [23:0] l1tag_valid_dirty; 
wire [7:0]  l1index;    
wire [63:0] l1data;        
wire        write_from_l2_to_l1;
wire [23:0] update_l1_tag_valid_dirty;                
wire [63:0] update_l1_data_from_l2;      
wire [63:0] from_cpu_to_l1_data_i;

wire write_cpu_data_to_l1_cache; 


// signals and arrays mostly related with l2 cache
wire [63:0]  l2_data;
wire [19:0]  l2tag_valid_dirty;
wire         l2_cache_req_ready;
wire [9:0]   l2index;
wire [31:0]  l2_cache_addr; // bu baðlý olmayabilir. 
wire         l2_cache_req_rw;
wire         l2_cache_req_valid;
wire [19:0]  update_l2_tag_valid_dirty;   
wire [63:0]  update_l2_data_from_main_memory;
wire         write_from_main_memory_to_l2_cache;



// signals and arrays mostly related with main memory
                            
wire mem_req_ready;  
//wire [63:0] main_memory_data_i;         
wire [63:0] main_memory_data;              
wire mem_req_rw;
wire mem_req_valid;
wire[31:0] mem_req_addr;

wire [63:0] main_memory_data_from_l2;

wire write_from_l1_to_l2; 


memory_controller memory_controller_instantiation (.clk(clk), .rst(rst), .cpu_req_read(cpu_req_read), .cpu_req_valid(cpu_req_valid), .cpu_req_write(cpu_req_write),
                        .cpu_req_address(cpu_req_address), .cpu_data_i(cpu_data_i), .cpu_data_o(cpu_data_o_), .cache_ready(cache_ready), 
                        
                        .l1tag_valid_dirty(l1tag_valid_dirty), .l1index(l1index), .l1data_i(l1data), .write_from_l2_to_l1(write_from_l2_to_l1),
                        .update_l1_tag_valid_dirty(update_l1_tag_valid_dirty), .update_l1_data_from_l2(update_l1_data_from_l2),   
                        .cpu_data_i_reg(from_cpu_to_l1_data_i) ,
                        .write_cpu_data_to_l1_cache(write_cpu_data_to_l1_cache), 
                        .after_rst_data_is_invalid_flag(after_rst_data_is_invalid_flag),
                        
                        .l2_data_i(l2_data), .l2tag_valid_dirty(l2tag_valid_dirty), .l2_cache_req_ready(l2_cache_req_ready), .l2index(l2index),
                        .l2_cache_addr(l2_cache_addr), .l2_cache_req_rw(l2_cache_req_rw) ,.l2_cache_req_valid(l2_cache_req_valid) ,.update_l2_tag_valid_dirty (update_l2_tag_valid_dirty),
                        .update_l2_data_from_main_memory(update_l2_data_from_main_memory) ,.write_from_main_memory_to_l2_cache(write_from_main_memory_to_l2_cache),
                        .write_from_l1_to_l2(write_from_l1_to_l2),
                        
                        .mem_req_addr(mem_req_addr) ,.mem_req_valid(mem_req_valid) ,.mem_req_rw(mem_req_rw) ,.mem_req_ready(mem_req_ready) ,.main_memory_data_i(main_memory_data),
                        .main_memory_data_from_l2(main_memory_data_from_l2), .data_from_l1_is_loaded(data_from_l1_is_loaded)    
                        );
                        
                        
l1_cache     l1_cache_instantiation(.clk(clk), .rst(rst), .cpu_addr(cpu_req_address), .l1data_o(l1data), .l1index(l1index), .l1tag_valid_dirty(l1tag_valid_dirty),
                                    .write_from_l2_to_l1(write_from_l2_to_l1), .update_l1_tag_valid_dirty(update_l1_tag_valid_dirty), .update_l1_data_from_l2(update_l1_data_from_l2),
                                    .data_i(from_cpu_to_l1_data_i) ,.write_cpu_data_to_l1_cache(write_cpu_data_to_l1_cache),
                                    .after_rst_data_is_invalid_flag(after_rst_data_is_invalid_flag)
                                    );


l2_cache     l2_cache_instantiation(.clk(clk),.rst(rst) , .cpu_addr(cpu_req_address) ,.l2index(l2index) ,.l2tag_valid_dirty(l2tag_valid_dirty) ,.write_from_main_memory_to_l2(write_from_main_memory_to_l2_cache)
                                    ,.update_l2_tag_valid_dirty(update_l2_tag_valid_dirty), .l2data_o(l2_data) ,.from_l1_data_i(from_cpu_to_l1_data_i), .update_l2_data_from_main_memory(update_l2_data_from_main_memory),
                                    .after_rst_data_is_invalid_flag(after_rst_data_is_invalid_flag),
                                    .l2_cache_req_ready(l2_cache_req_ready), .write_from_l1_to_l2(write_from_l1_to_l2),
                                    .data_from_l1_is_loaded(data_from_l1_is_loaded));


main_memory  main_memory_instantiation(.clk(clk), .rst(rst), .mem_addr(mem_req_addr), .mem_req_ready(mem_req_ready) ,.main_memory_data_o(main_memory_data) ,.mem_req_valid(mem_req_valid) ,.mem_req_rw(mem_req_rw) ,.main_memory_data_from_l2(main_memory_data_from_l2) );
         
                          
endmodule
