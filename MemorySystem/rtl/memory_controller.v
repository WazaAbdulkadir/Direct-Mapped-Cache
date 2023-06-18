`timescale 1ns / 1ps

module memory_controller (
                            input clk,
                            input rst,
                            
                            // cpu signals
                            input cpu_req_valid,
                            input cpu_req_read,
                            input cpu_req_write, 
                            input [31:0] cpu_req_address,
                            
                            input [63:0] cpu_data_i,
                            output reg [63:0] cpu_data_o,
                            
                            // cache to cpu signal to let cpu know it can take new request.
                            output reg cache_ready,
                            
                            // rst olduğu zaman cachelerde valid bit 0 lanmalı 
                           // output reg rst_flag, 
                            
                            //input wire [21:0]l1tag,
                            input [23:0]     l1tag_valid_dirty,
                            input wire [7:0] l1index,
                            input [63:0]     l1data_i,
               
                  // l2den l1 e yazma ve l1 in tag valid dirty bitlerini update etme
                            output reg        write_from_l2_to_l1,
                            output reg [23:0] update_l1_tag_valid_dirty,                      
                            output reg [63:0] update_l1_data_from_l2, 
                            output reg after_rst_data_is_invalid_flag, 
                            
                            output reg  write_cpu_data_to_l1_cache,
                            output reg [63:0] cpu_data_i_reg, 
                            
               
                 //L2: 2 bit offset + 2 bit block select + 10 bit index + 18 bit tag
                           
                           // input [255:0] l2_data_i,
                            input [63:0]  l2_data_i,
                            input [19:0]  l2tag_valid_dirty,
                            
                            //input wire [17:0]l2tag,   
                            
                            
                            // bu sinyal bağlı değil. 
                            output reg        l2_cache_req_valid,
                           
                            input             l2_cache_req_ready,
                            input      [9:0]  l2index,
                            output reg [31:0] l2_cache_addr,
                            output reg        l2_cache_req_rw,
                            
                            output reg [19:0] update_l2_tag_valid_dirty,   
                            output reg [63:0] update_l2_data_from_main_memory,
                           // output reg [63:0] 
                            output reg        write_from_main_memory_to_l2_cache,
                            output reg        write_from_l1_to_l2, 
                            input data_from_l1_is_loaded,
                            
                            
                               
                            
                            input mem_req_ready,   
                            input [63:0] main_memory_data_i,
                            // main memory ye giden sinyaller
                           
                            output reg mem_req_rw,
                            output reg mem_req_valid,
                            output reg [31:0] mem_req_addr,
                            
                            output reg [63:0] main_memory_data_from_l2
                       
                            );
                            
                      

reg rst_flag;

reg [31:0] cpu_req_addr_reg, next_cpu_req_addr;                                
reg [63:0] next_cpu_data_i;  
reg        cpu_req_read_reg,   next_cpu_req_read;
reg        cpu_req_write_reg,  next_cpu_req_write;   
reg        cache_ready_reg,  next_cache_ready;     

//reg write_from_main_memory_to_l2_cache;     

reg [31:0] next_l2_cache_addr;
reg next_l2_cache_req_rw;
reg next_l2_cache_req_valid;

reg [31:0] next_mem_req_addr; 
reg next_mem_req_rw;
reg next_mem_req_valid;

//reg [63:0] splitted_64bit_l2_cache_data; 

reg l1_data_update;
reg l1_tag_update;
reg l1_valid_bit;
reg l1_dirty_bit; 

reg l2_valid_bit;
reg l2_dirty_bit;            
reg l2_tag_update;       
reg l2_data_update;

reg write_old_l1_data_to_l2;
reg l2_data_update_from_l1; 
reg l1_is_ready_to_take_new_cpu_data; 

//reg [23:0] previous_l1tag_valid_dirty;
reg l2_tag_update_when_l1_dirty; 
reg [17:0] old_l1_tag;  // l2 tag ile compare edilcek sanırım. 

reg [17:0] old_l2_tag;
reg write_old_l2_data_to_main_memory; 
reg [63:0] next_main_memory_data_from_l2;

reg memory_is_allocated_flag; 
reg old_l1_data_loaded_to_l2;                
reg requested_data_is_in_l2_flag;                
               
reg [2:0] present_state, next_state;                 
parameter IDLE      = 3'b000, 
          l1COMPARE = 3'b001,
          l2COMPARE = 3'b010,
          ALLOCATE  = 3'b011,
          WRITEBACK = 3'b100;        
          
wire [21:0] cpu_compare_l1tag;
wire [7:0]  cpu_compare_l1index;
wire [1:0]  cpu_addr_offset;

wire [17:0] cpu_compare_l2tag;
wire [9:0]  cpu_compare_l2index;
wire [1:0]  l2_block_place;
//wire [1:0]  cpu_addr_offset;
    
wire L1hit;
wire L2hit;        
                                                 
// 22 bit L1 tag , 18 bit L2 tag
assign cpu_compare_l1tag   = cpu_req_addr_reg[31:10] ;
assign cpu_compare_l1index = cpu_req_addr_reg[9:2];

assign cpu_addr_offset     = cpu_req_addr_reg[1:0];


assign cpu_compare_l2tag   =   cpu_req_addr_reg[31:14]; // 18 bit l2 tag 
assign cpu_compare_l2index =   cpu_req_addr_reg[13:4];
assign l2_block_place     =   cpu_req_addr_reg[3:2];
//assign cpu_addr_offset     = cpu_req_addr_reg[1:0];
 
// valid ise ve 22 bitlik tag uyuşuyorsa hit.  
assign L1hit = l1tag_valid_dirty[23] && (cpu_compare_l1tag ==l1tag_valid_dirty[21:0]);

// l2 hit olması set associative mantığına göre ayarlanacak. 
assign L2hit = l2tag_valid_dirty[19] && (cpu_compare_l2tag ==l2tag_valid_dirty[17:0]); 

 
// output reset and output assign 
 always@(posedge clk) begin
       
       
       
      if(rst) begin
         
                
          //  rst_flag <= 1'b1; 
            update_l1_tag_valid_dirty <= 24'b0;
            update_l1_data_from_l2 <= 64'b0; 
            update_l2_tag_valid_dirty <= 20'b0;
            update_l2_data_from_main_memory <= 63'b0;
            after_rst_data_is_invalid_flag <= 1'b0; 
                
             mem_req_rw <= 1'b0;     
             old_l1_tag <= 18'b0; 
       end
       
       
       
//       else if (rst_flag) begin
//            update_l1_tag_valid_dirty <= {l1_valid_bit, l1_dirty_bit, 22'b0}; 
//            update_l2_tag_valid_dirty <=   {l2_valid_bit, l2_dirty_bit, 18'b0};
//            rst_flag <= 1'b0; 
            
//            // this flag will be sended to l1 cache and rst the data inside corresponding 
//            after_rst_data_is_invalid_flag <= 1'b1; 
//       end 
       
       
   
       
       
       else begin
            
            
            cpu_req_addr_reg            <= next_cpu_req_addr;
            cpu_data_i_reg              <= next_cpu_data_i; 
            l2_cache_addr               <= next_l2_cache_addr;
            l2_cache_req_rw             <= next_l2_cache_req_rw;
            l2_cache_req_valid          <= next_l2_cache_req_valid;
            
            cpu_req_read_reg             <= next_cpu_req_read;
            cpu_req_write_reg <= next_cpu_req_write;
            
            cache_ready                 <= next_cache_ready;
            
            // update arrayi eğer "write_from_l2_to_l1" sinyali aktifse güncellenir. 
            update_l1_tag_valid_dirty        <=   l1_tag_update  ? {l1_valid_bit,l1_dirty_bit,cpu_compare_l1tag} : update_l1_tag_valid_dirty;
           // update_l1_data_from_l2           <=  l1_data_update ?  splitted_64bit_l2_cache_data :  update_l1_data_from_l2;  
            update_l1_data_from_l2           <=  l1_data_update ?  l2_data_i :  update_l1_data_from_l2;  
            
            update_l2_tag_valid_dirty        <=  l2_tag_update  ? {l2_valid_bit, l2_dirty_bit, cpu_compare_l2tag} : l2_tag_update_when_l1_dirty ?  {l2_valid_bit, l2_dirty_bit, old_l1_tag} : update_l2_tag_valid_dirty;
            update_l2_data_from_main_memory  <=  l2_data_update ? main_memory_data_i : l2_data_update_from_l1 ?  l1data_i : update_l2_data_from_main_memory ;
      
           // update_l2_tag_valid_dirty 
            mem_req_rw   <= next_mem_req_rw; 
            mem_req_addr  <= next_mem_req_addr;
            mem_req_valid <= next_mem_req_valid;
            
            after_rst_data_is_invalid_flag <= 1'b0; 
            
       end 
    
 end              
 
 always@ (posedge clk) begin
            
         if (rst) begin 
          //  rst_flag  <= 1'b1; 
            present_state <= IDLE;  
            l1_tag_update    <= 1'b0; 
            l1_valid_bit     <= 1'b0;
            l1_data_update   <= 1'b0;
            l1_dirty_bit     <= 1'b0;
            
            l2_valid_bit     <= 1'b0; 
            l2_dirty_bit     <= 1'b0;            
            l2_tag_update    <= 1'b0;       
            l2_data_update   <= 1'b0;
            
            next_cache_ready <= 1'b1;
            
            write_from_main_memory_to_l2_cache <= 1'b0;
            write_from_l2_to_l1                <= 1'b0;    
            
            write_old_l1_data_to_l2 <= 1'b0;
            write_old_l2_data_to_main_memory <= 1'b0; 
            
            next_state       <=  3'b000; 
            
            write_cpu_data_to_l1_cache <= 1'b0;
            l2_tag_update_when_l1_dirty<= 1'b0;
            l2_data_update_from_l1     <= 1'b0;  
            
            memory_is_allocated_flag  <= 1'b0; 
            
            main_memory_data_from_l2   <= next_main_memory_data_from_l2;
            write_from_l1_to_l2 <= 1'b0; 
            requested_data_is_in_l2_flag <= 1'b0;
            old_l1_data_loaded_to_l2 <= 1'b0;
            l1_is_ready_to_take_new_cpu_data <= 1'b0; 
            cpu_data_o <= 64'b0; 
         end   

//            case (l2_block_place)
//                2'b00: splitted_64bit_l2_cache_data   <= l2_data_i[63:0];
//                2'b01: splitted_64bit_l2_cache_data   <= l2_data_i[127:64];
//                2'b10: splitted_64bit_l2_cache_data   <= l2_data_i[191:128];
//                2'b11: splitted_64bit_l2_cache_data   <= l2_data_i[255:192];
//                default: splitted_64bit_l2_cache_data <= 64'd0;
//            endcase
        else begin
         present_state  <= next_state;
           // rst_flag <= 1'b0; 
            case(present_state) 
                IDLE: begin : idle
                
                    
//                    l1_valid_bit     <= 1'b0;
                   // l1_dirty_bit     <= 1'b0;
//                    l1_tag_update       <= 1'b0; 
//                    l1_data_update      <= 1'b0;
                    
//                    l2_valid_bit     <= 1'b0; 
                    //l2_dirty_bit     <= 1'b0;            
//                    l2_tag_update    <= 1'b0;       
//                    l2_data_update   <= 1'b0;
                    
//                    write_cpu_data_to_l1_cache <=1'b0;
//                    l2_tag_update_when_l1_dirty <= 1'b0;
                    
                    
                    if(cpu_req_valid) begin
                       // present_state  <= next_state;
                        next_cpu_req_addr <= cpu_req_address;
                        next_cpu_data_i   <= cpu_data_i; 
                        next_cache_ready  <= 1'b0;
                        l1_tag_update  <= 1'b0; 
                        l1_data_update <= 1'b0;
                        write_cpu_data_to_l1_cache <= 1'b0; 
                        write_from_l1_to_l2 <= 1'b0; 
                        if(cpu_req_read) begin
                            next_cpu_req_read  <= cpu_req_read;  
                            next_cpu_req_write <= cpu_req_write; 
                        end
                        
                        else if (cpu_req_write) begin
                            next_cpu_req_write <= cpu_req_write; 
                            next_cpu_req_read  <= cpu_req_read;  
                        end     
                        next_state        <= l1COMPARE; 
                        
                    
                    end
                    
                    

                    else 
                        present_state <= IDLE;
                    
                end : idle 
                
              l1COMPARE: begin: l1compare 
                    //present_state  <= next_state;    
                    next_cache_ready  <= 1'b0;
                    l1_tag_update     <= 1'b0; 
                    l1_data_update    <= 1'b0; 
                    
                    l2_data_update_from_l1 <= 1'b0; 
                    // read operation hit 
                    if(L1hit && cpu_req_read_reg)begin
                      // l1datacache den veriyi al outputa ver.  
                     write_from_l2_to_l1 <= 1'b0;
                     cpu_data_o <= l1data_i;
                     next_cache_ready <= 1'b1;
                     next_state <= IDLE;
                    end 
                    
                    else if (!L1hit && cpu_req_read_reg) begin
                        next_cache_ready  <= 1'b0;
                           // 
                           if ( (!l1tag_valid_dirty[22] && !L2hit )) begin // dirty biti 0 ise 
                                
                                // eğer l2 de karşılık gelen yer boş ise l2 ye yazılacak tag 
                                //next_l2_cache_tag        <= cpu_req_addr_reg[31:14];
                                
                                next_l2_cache_req_rw     <= 1'b0;
                                next_l2_cache_req_valid  <= 1'b1;
                                
                                next_state               <= l2COMPARE; 
                          end      
                                
                          // bu kısım eklenecek       
                           else if (L2hit )begin  // dirty biti 1 ise  
                                
                                requested_data_is_in_l2_flag <= 1'b1; 
                                
                                next_l2_cache_req_rw     <= 1'b0;
                                next_l2_cache_req_valid  <= 1'b1;
                                
                                next_state               <= l2COMPARE; 
                           end


                    else begin 
                        next_state <= present_state;
                    end 
                
                    end   
                    
                    // write operation 
                    else if ( (cpu_req_write_reg && !l1tag_valid_dirty[22]) | l1_is_ready_to_take_new_cpu_data) begin // write operation with dirty bit 0 
                        
                        l1_valid_bit   <= 1'b1; 
                        l1_dirty_bit   <= 1'b1;
                        l1_tag_update  <= 1'b1; 
                        l1_data_update <= 1'b1; 
                        write_cpu_data_to_l1_cache <= 1'b1; 
                        
                        l1_is_ready_to_take_new_cpu_data <= 1'b0;  
                        l2_tag_update_when_l1_dirty <= 1'b0;
                        l2_tag_update_when_l1_dirty <= 1'b0; 
                        next_state <= IDLE; 
                    
                    end     
                    
                    
                    else if (cpu_req_write_reg && l1tag_valid_dirty[22] && !old_l1_data_loaded_to_l2 ) begin
                        
                        write_old_l1_data_to_l2 <= 1'b1;   
                        old_l1_tag              <= {l1tag_valid_dirty[21:3]};
                        l1_tag_update  <= 1'b0; 
                        l1_data_update <= 1'b0;
                                              
                                          
                        next_state <= l2COMPARE; 
                        
                    
                    end 
                    
                    
                    else if (cpu_req_write_reg && old_l1_data_loaded_to_l2)  begin
                                 l1_dirty_bit <= 1'b0; 
                                l1_valid_bit <= 1'b1; 
                                l1_tag_update  <= 1'b1; 
                                //l1_data_update <= 1'b1; 
                                old_l1_data_loaded_to_l2 <= 1'b0;
                                l1_is_ready_to_take_new_cpu_data <= 1'b1; 
                                next_state <= l1COMPARE; 
                    end 
                    else begin
                       // next_state <= present_state;
                       present_state <= next_state; 
                    end
                               
              end: l1compare  
                        
              l2COMPARE: begin : l2compare
              
                    //present_state  <= next_state;
                    next_l2_cache_req_valid <= 1'b0;
                    next_cache_ready        <= 1'b0;
                    
                    // allocate de update ediliyor daha sonra allocate den buraya geliniyor.
                    // burada tekrar lowa çekiyorum. 
                    l2_tag_update       <= 1'b0; 
                    l2_data_update      <= 1'b0;
                    
                    
                   // if(L2hit && !l2_cache_req_valid && l2_cache_req_ready ) // wait for l2 cache to be ready    
                   if((L2hit && l2_cache_req_ready && memory_is_allocated_flag) || requested_data_is_in_l2_flag)  
                       begin
                            write_from_l2_to_l1 <= 1'b1; // l2 cache ten gelen veriyi
                                                         // l1 cache e yaz 
                            write_from_main_memory_to_l2_cache <= 1'b0; 
                            
                            // valid ve dirty bitler tag_array e yazılmalı. 
                            l1_valid_bit        <= 1'b1;
                            l1_dirty_bit        <= 1'b0; // data zaten l2 de, l1 dirty biti 0 olarak kalabilir.             
                            l1_tag_update       <= 1'b1; 
                            l1_data_update      <= 1'b1;
                            // l1compare olmayabilir. 
                            next_state          <= l1COMPARE; 
                            
                            memory_is_allocated_flag <= 1'b0; 
                            requested_data_is_in_l2_flag <= 1'b0;
                            
                            
                       end               
                   // l2 henüz hit olmamışsa ama main memoryden l2 cache veri akışı sağlanmışsa 
                
                     
 // l2 hit yapmazsa; okuma isteğiyse VE l2 clean ise main memory ye çık veriyi l2 ya yaz sonra l1 e gider
                    else if (cpu_req_read_reg && !l2_cache_req_ready && !L2hit && !memory_is_allocated_flag ) begin
                       // next_cache_ready <= 1'b0;
                           if (!l2tag_valid_dirty[18]) begin // L2 temiz ise 
                                
                                next_mem_req_addr   <= cpu_req_addr_reg;
                                next_mem_req_rw     <= 1'b0; 
                                next_mem_req_valid  <= 1'b1; 
                                // main memory ye çık l2 ye yaz. 
                                next_state          <= ALLOCATE; 
                                                           
                           end  
                           
                     end       
                     
                         
                            
                       // next_state <= ALLOCATE;
                       

                    // yazma isteği 
                    else if (cpu_req_write_reg ) begin
                        //    next_cache_ready <= 1'b0;
                            if (!l1tag_valid_dirty[22]) begin // l1 deki eski data zaten l2 deyse 
                             
//                             write_old_l1_data_to_l2 <= 1'b0;
//                             l2_dirty_bit <= 1'b0;
//                             l2_valid_bit <= 1'b1; 
                             
//                             l1_dirty_bit <= 1'b0; 
                             
//                             l2_tag_update_when_l1_dirty <= 1'b1;
//                             l2_data_update_from_l1      <= 1'b1; 
                                
                            l1_valid_bit        <= 1'b1;
                            l1_dirty_bit        <= 1'b1; // data zaten l2 de, l1 dirty biti 0 olarak kalabilir.             
                            l1_tag_update       <= 1'b1; 
                            l1_data_update      <= 1'b1;
                             
                             next_state <= l1COMPARE; 
                             
                             
                            end     
                            
                     // eski l1 l2 ye yüklenince l2 den bir sinyal gönder ve yeni veriyi l1 e al        
                            else if ((l1tag_valid_dirty[22]) && (write_old_l1_data_to_l2 && !l2tag_valid_dirty[18] )) begin // l2 dirty değilse ve l1 deki eski veri ile l2 deki veri uyuşmuyorsa. l1 deki eski veriyi l2 ye yaz. 
                                 l2_dirty_bit <= 1'b1;
                                 l2_valid_bit <= 1'b1; 
                             
                                
                                
                                write_from_l1_to_l2 <= 1'b1;  
                               // write_old_l1_data_to_l2 <= 1'b0; 
                                l2_tag_update_when_l1_dirty <= 1'b1;
                                l2_data_update_from_l1      <= 1'b1; 
                               
                               
                               if (data_from_l1_is_loaded) begin
                                    write_from_l1_to_l2 <= 1'b0;
                                    old_l1_data_loaded_to_l2 <= 1'b1; 
                                    next_state <= l1COMPARE;
                                    write_old_l1_data_to_l2 <= 1'b0; 
                                    
                               end 
                               
                               else begin
                                    next_state <= present_state; 
                               end 
                               
                                    
                                    
                            end  
                            
                            else if (write_old_l1_data_to_l2 && l2tag_valid_dirty[18] ) begin // l2 deki eski datayı main memoryye yaz daha sonra l1 deki datayı l2 ye yaz sonra yeni datayı l1 e yaz. 
                              
                                write_old_l2_data_to_main_memory <= 1'b1; 
                                next_mem_req_addr <= {l2tag_valid_dirty,cpu_compare_l2index, 2'b00 };
                                next_mem_req_rw   <= 1'b1; 
                                next_mem_req_valid <= 1'b1;
                                next_main_memory_data_from_l2 <= l2_data_i;
                                next_state <= WRITEBACK; 
                                    
                            end 
                    
                    end 
                    
                    // l1 miss yaptı ama l2 de hit yapıldı. 
//                    else if (L2hit && !L1hit) begin
                        
//                            write_from_l2_to_l1 <= 1'b1; // l2 cache ten gelen veriyi
//                                                         // l1 cache e yaz 
                            
//                            // valid ve dirty bitler tag_array e yazılmalı. 
//                            l1_valid_bit        <= 1'b1;
//                            l1_dirty_bit        <= 1'b0; // data zaten l2 de, l1 dirty biti 0 olarak kalabilir.             
//                            l1_tag_update       <= 1'b1;
//                            l1_data_update      <= 1'b1;
//                            // l1compare olmayabilir. 
//                            next_state          <= l1COMPARE; 
                    
//                    end 
                        
                    else begin
                       // next_state <= present_state; 
                       present_state <= next_state; 
                     end   
            
              end: l2compare
                      
            ALLOCATE : begin : allocate
                  //present_state  <= next_state;
                  next_mem_req_valid <= 1'b0;
                  next_cache_ready   <= 1'b0; 
                    if(!mem_req_rw && mem_req_valid && mem_req_ready) begin  // memory den oku 
                        
                        write_from_main_memory_to_l2_cache <= 1'b1; 
                        
                        l2_valid_bit        <= 1'b1;
                        l2_dirty_bit        <= 1'b0; // data main memory de, dirty 0 olur.              
                        l2_tag_update       <= 1'b1; 
                        l2_data_update      <= 1'b1;
                        
                        next_state          <= l2COMPARE; 
                        
                        memory_is_allocated_flag <= 1'b1; 
                        
                    
                    end
                    else begin
                       present_state <= next_state; 
                    end
            
            end: allocate 
            
            
            WRITEBACK: begin : writeback
                   // present_state  <= next_state;
                    next_mem_req_valid <= 1'b0;
                    next_cache_ready   <= 1'b0;
                    
                    if (!mem_req_valid && mem_req_ready && write_old_l2_data_to_main_memory) // l2 kirli verisi main memory ye yazıldıysa ve memoryden veri okunmaya hazırsa 
                        begin
                            
                            l2_valid_bit  <= 1'b0; 
                            l2_dirty_bit  <= 1'b0;
                            l2_tag_update <= 1'b1;
                            
                            write_old_l2_data_to_main_memory <= 1'b0; 

                            next_state    <= l2COMPARE;   
                            
                        end 
                        
                    else begin
                        present_state <= next_state; 
                    end     
                
            end : writeback 
            
  
            endcase
        end
     end                       
endmodule 