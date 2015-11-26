module Processing_logic(
   // Outputs
   DATA_get, 
   CMD_get,
   RETURN_put, 
   RETURN_address, RETURN_data,  //construct RETURN_data_in
   cs_bar, ras_bar, cas_bar, we_bar,  // read/write function
   BA, A, DM,
   DQS_out, DQ_out,
   ts_con,
   // Inputs
   clk, ck, reset, ready, 
   CMD_empty, CMD_data_out, DATA_data_out,
   RETURN_full,
   DQS_in, DQ_in
   );

   parameter BL = 4'd8; // Burst Lenght = 8
   parameter BT = 1'd0;   // Burst Type = Sequential
   parameter CL = 3'd5;  // CAS Latency (CL) = 5
   parameter AL = 3'd3;  // Posted CAS# Additive Latency (AL) = 3

   //State
   parameter  INIT = 4'b0000, PRECHARGE = 4'b0001, IDLE = 4'b0010, ACTIVE = 4'b0011, REFRESH = 4'b0100, 
              SCALARREAD = 4'b0101, SCALARWRITE = 4'b0110, BLOCKREAD = 4'b0111, BLOCKWRITE = 4'b1000, ATOMICREAD = 4'b1001, ATOMICWRITE = 4'b1010;
   parameter  NOT = 3'b000, NAND =3'b001, NOR = 3'b010, XOR = 3'b011, AND = 3'b100, FLIP = 3'b101, SRA = 3'b110, SLA = 3'b111;
   
   parameter tRCD = 4'd10; //row to column latency
   parameter tCK = 2'd2; // 2 system clocks for tCK  clk*2
   
   //Read PARA
   parameter NOP1 = tCK; // issue nop after activate
   parameter tRDissue = tRCD-AL*2;//issue read command
   parameter NOP2 = tRDissue+2;//issue nop after read
   parameter tLISTEN = tRDissue+(AL+CL-1)*2;//issue listen
   parameter tDQrd = tLISTEN+2; //DQ arrive

   // Write PARA
   parameter NOP3 = tCK;  //issue nop after activate 
   parameter tWRTissue = tRCD-AL*2; //issue write command
   parameter NOP4 = tWRTissue+2;  //issue nop after write command
   parameter tDQwrt=tWRTissue+(AL+CL)*2; //DQ arrive
   
   input 	 clk, ck, reset, ready;
   input 	 CMD_empty, RETURN_full;
   input [33:0]	 CMD_data_out;
   input [15:0]  DATA_data_out;
   input [15:0]  DQ_in;
   input [1:0]   DQS_in;
 
   output reg CMD_get;
   output reg	DATA_get, RETURN_put;
   output reg [25:0] RETURN_address;
   output wire [15:0] RETURN_data;
   output reg	cs_bar, ras_bar, cas_bar, we_bar;
   output reg [2:0]	 BA;
   output reg [13:0] A;
   output reg [1:0]	 DM;
   output reg [15:0]  DQ_out;
   output reg [1:0]   DQS_out;
   output reg ts_con;
   

   reg listen;

   reg DM_flag;

   reg [2:0] Pointer;
   reg [3:0] state;
   reg [12:0] count_ref;
   reg [6:0] count_rd;
   reg [6:0] count_wrt;
   reg [25:0] rd_addr;
   reg [25:0] wrt_addr;

   reg [4:0] count_NOP;
   reg [1:0] count_dataget;
   reg flag_DQSstart, flag_DQSadd, flag_DQSend;

   reg [2:0] OP;

   reg [15:0] RETURN_data_temp1;
   reg [15:0] RETURN_data_temp;
   reg [15:0] DATA_data_out_temp;
   
   reg flag_automic;

always @(negedge clk) begin
    if(reset)
	   begin
	      state <= INIT;
        CMD_get <= 0;
        DATA_get <= 0;
        RETURN_put <= 0;
        {cs_bar,ras_bar,cas_bar,we_bar} <=  4'b0111; 
        //DQ_out <= 0;
        //DQS_out <= 0;
        ts_con <= 0;
        listen <= 0;
        DM_flag <= 0;
        Pointer <= 0;
        count_rd <= 0;
        count_wrt <= 0;
        count_ref <= 0;
        count_NOP <= 0;
        count_dataget <= 0;
        flag_DQSstart <= 0;
        flag_DQSadd <= 0;
        flag_DQSend <= 0;
        rd_addr <= 0;
        wrt_addr <= 0;
	  end
	  else begin
      if(state != INIT) begin
         count_ref <= count_ref + 1;
       end 
      
      case (state)
        INIT: begin
          state <= ready?PRECHARGE:INIT;
        end

        PRECHARGE: begin
            if(count_NOP == 5'd0) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0010 ; // Precharge all banks command
              A[10] <= 1'b1;
              count_NOP <= count_NOP + 1;
            end
            else if(count_NOP == 5'd2) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              count_NOP <= count_NOP + 1;
            end
            else if(count_NOP == 5'd16) begin
              state <= IDLE;
              count_NOP <= 0;
            end
            else begin
              count_NOP <= count_NOP + 1;
            end
          end
        
        IDLE: begin
          if(count_ref == 13'd4000) begin
            state <= REFRESH;
          end
          else begin
            if(CMD_empty && count_NOP == 0) begin
              state <= IDLE;
              count_NOP <= 0;
            end
            else if(count_NOP == 2) begin
              if (~CMD_empty && ~(CMD_data_out[7:5] == 3'b001 && RETURN_full == 1'b1)) begin
              CMD_get <= 1;
              end
              count_NOP <= count_NOP + 1;
            end
            else if(count_NOP == 4) begin
              state <= ACTIVE;
              count_NOP <= 0;
            end
            else begin
              count_NOP <= count_NOP + 1;
              state <= IDLE;
              CMD_get <= 0;
            end
          end
        end

        ACTIVE: begin
          if(CMD_data_out[7:5]==3'b000) begin //Idle
              state <= IDLE;
              count_NOP <= 0;
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
            end
                
          else if(CMD_data_out[7:5] == 3'b001) begin
              state <= ACTIVE;
              rd_addr <= CMD_data_out[33:8];
              if (count_NOP <= 1) begin
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011; //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= SCALARREAD;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
            end
                
          else if(CMD_data_out[7:5] == 3'b010) begin
              state <= ACTIVE;
              wrt_addr <= CMD_data_out[33:8];
              if (count_NOP <= 1) begin
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011;  //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= SCALARWRITE;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
            end
          
          else if (CMD_data_out[7:5] == 3'b011) begin
              if (count_NOP == 0) begin
                rd_addr <= CMD_data_out[33:8];
                size <= CMD_data_out[4:3];
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011; //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= BLOCKREAD;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
              else begin
                count_NOP <= count_NOP + 1;
              end
            end          

          else if (CMD_data_out[7:5] == 3'b100) begin
              if (count_NOP == 0) begin
                wrt_addr <= CMD_data_out[33:8];
                size <= CMD_data_out[4:3];
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011;  //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= BLOCKWRITE;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
              else begin
                count_NOP <= count_NOP + 1;
              end
            end

          else if(CMD_data_out[7:5] == 3'b101) begin
                if (count_NOP == 0) begin
                rd_addr <= CMD_data_out[33:8];
                OP <= CMD_data_out[2:0];
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011; //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= ATOMICREAD;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
              else begin
                count_NOP <= count_NOP + 1;
              end
            end
          
          else if(CMD_data_out[7:5] == 3'b110) begin
            if (count_NOP == 0) begin
                wrt_addr <= CMD_data_out[33:8];
                OP <= CMD_data_out[2:0];
                {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0011; //bank active
                A <= CMD_data_out[30:18];
                BA <= CMD_data_out[33:31];
                count_NOP <= count_NOP + 1;
              end
              else if(count_NOP == 2) begin
                state <= ATOMICWRITE;
                count_NOP <= 0;
                {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              end
              else begin
                count_NOP <= count_NOP + 1;
              end
            end

          else begin
              A <= CMD_data_out[30:18];
              BA <= CMD_data_out[33:31];
              state <= ACTIVE;
              count_NOP <= count_NOP + 1;
          end
        end

        REFRESH: begin
            if(count_NOP == 3'd0) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0001; //refresh
              count_ref <= 0;
              count_NOP <= count_NOP + 1;
            end
            else if(count_NOP == 3'd2) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111 ;
              count_NOP <= count_NOP + 1;
            end
            else if(count_NOP == 3'd3) begin
              state <= IDLE;
              count_NOP <= 0;
            end
            else begin
              count_NOP <= count_NOP + 1;
            end
        end

        SCALARREAD: begin
            if(count_rd==0) begin 
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111; // NOP
              end
            else if(count_rd==tRDissue) begin  //
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0101; // Read
              A[9:0] <= rd_addr[9:0]; //column address
              A[10] <= 0; //auto precharge 
              end
            else if(count_rd < NOP2) begin
              A[9:0] <= rd_addr[9:0]; //column address
              A[10] <= 0; //auto precharge 
            end
            else if(count_rd==NOP2) begin
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111; // NOP
              end
            else if(count_rd==tLISTEN) begin
              listen <= 1;
              end
            
            else if(count_rd == tDQrd) begin
              Pointer <= 0;
              ts_con <= 0;
              listen <= 0;
              end
            else if(count_rd==(tDQrd+1)) begin
              RETURN_address <= rd_addr;
              end
            else if(count_rd==(tDQrd+2)) begin
              RETURN_put <= 1;
              end
            else if(count_rd==(tDQrd+3)) begin
              RETURN_put <= 0;
              end
            
            if(count_rd==(tDQrd+BL+6)) begin
              state <= PRECHARGE;
              count_rd <= 0;
              end
            else begin
              state <= SCALARREAD;
              count_rd <= count_rd + 1;
              end
          end

        SCALARWRITE: begin
            ts_con <= 1;
            if(count_wrt==0) begin 
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111; // NOP
              end
            else if(count_wrt==tWRTissue) begin 
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0100; //Write
              A[9:0] <= wrt_addr[9:0]; //column address
              A[10] <= 0; //auto precharge
              end 
            else if(count_wrt < NOP4) begin
              A[9:0] <= wrt_addr[9:0]; //column address
              A[10] <= 0; //auto precharge
            end
            else if(count_wrt==NOP4) begin  
              {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111; //NOP
              end
            else if(count_wrt==tDQwrt-2) begin 
              flag_DQSstart <= 1;
              flag_DQSadd <= 0;
              flag_DQSend <= 0;
              end
            else if(count_wrt==tDQwrt-1) begin
              DATA_get <= 1;
              end
            else if(count_wrt==tDQwrt) begin 
              DATA_get <= 0;
              end
            
            if(count_wrt >= (tDQwrt) &&  (tDQwrt+BL) >= count_wrt) begin 
              flag_DQSend <= 0;
              flag_DQSstart <= 0;
              flag_DQSadd <= 1;
              end

            if (count_wrt == tDQwrt-1+wrt_addr[2:0]) begin
              DM_flag <= 1;
            end else if (count_wrt == tDQwrt+wrt_addr[2:0]) begin
              DM_flag <= 0;
            end

            if(count_wrt == (tDQwrt+BL)) begin
                flag_DQSend <= 1;
                flag_DQSstart <= 0;
                flag_DQSadd <= 0;

                count_dataget <= 0;
                if(count_NOP == 4'd12) begin
                  state <= PRECHARGE;
                  count_wrt <= 0;
                  count_NOP <= 0;
                end
                else begin
                  {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111;
                  count_NOP <= count_NOP + 1;
                end
              end
            else begin
              state <= SCALARWRITE;
              count_wrt <= count_wrt + 1;
              end 
          end        

       BLOCKREAD: begin
            //ts_con <= 0;
            if (count_rd == 0) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
              count_rd <= count_rd + 1;
            end
            else begin
               //if(size == 0) begin
                 if(count_rd == 6'd2) begin
                   {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0101;
                   A[9:0] <= rd_addr[9:0]; //col addr
                   A[10] <= 0; 
                   BA <= rd_addr[25:23];
                 end
                 else if(count_rd == 6'd4) begin
                   {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
                 end
                 else if(count_rd == 6'd10) begin
                   listen <= 1;
                 end
                 else if(count_rd == 6'd12) begin
                   //RETURN_put <= 1;  
                   listen<= 0;
                   ts_con <= 0;
                   Pointer <= 0;
                 end
                 else if(count_rd == 6'd13) begin
                   RETURN_put <= 1;
                 end
                // else if(count_rd == 6'd14) begin
                //   RETURN_put <= 0;
                 // end
                 else if(count_rd == 6'd20) begin
                   if(size != 0) begin
                     if(count_NOP == 0) begin
                       RETURN_put <= 0;
                       Pointer <= Pointer + 1;
                     end
                     else if(count_NOP == (BL/2)) begin
                        A[9:0] <= A[9:0] + 8;
                        size <= size - 1;
                        count_rd <= 0;
                        count_NOP <= 0;  
                     end
                     else begin
                       count_NOP <= count_NOP + 1;
                     end
                   end
                   else begin
                   RETURN_put <= 0; //close
                   count_rd <= 0 ;
                   state <= PRECHARGE;
                   Pointer <= Pointer + 1;
                   end
                 end
                 else begin
                   state <= BLOCKREAD;
                   count_rd <= count_rd + 1; 
                 end

                 if (count_rd > 2'd12 && count_rd < 2'd20) begin 
                    rd_addr <= rd_addr + 1;
                    Pointer <= Pointer + 1;
                    RETURN_address <= rd_addr;
                 end
               //end 
            end          
            //count_rd = count_rd + 1;

          end

        BLOCKWRITE: begin
            ts_con <= 1;
            if (count_wrt == 0) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
              count_wrt <= count_wrt + 1;  
            end
            else begin
              //if(size == 0) begin
                if(count_wrt == 6'd2) begin
                  {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0100;//write command, AL=1
                  A[9:0] <= wrt_addr[9:0]; //col addr
                  A[10] <= 1; //auto precharge when block = 8 words
                  BA <= wrt_addr[25:23];
                end
                else if(count_wrt == 6'd4) begin
                  {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
                end
                else if(count_wrt == 6'd8) begin
                  flag_DQSstart <= 1;
                  flag_DQSadd <= 0;
                  flag_DQSend <= 0; 
                end
                else if(count_wrt == 6'd9) begin
                  DATA_get <= 1; 
                end
                else if(count_wrt == 6'd10) begin
                  DATA_get <= 0; 
                end
                else if(count_wrt == 6'd16) begin
        
                  DATA_get <= 0;  
                end
                else if(count_wrt == ) begin
                  ts_con <= 0;
                end
                else if(count_wrt == ) begin
                  if(size != 0) begin
                    if(count_NOP == (BL/2)) begin
                        A[9:0] <= A[9:0] + 8;
                        size <= size - 1;
                        count_wrt <= 0;
                        count_NOP <= 0;  
                     end
                     else begin
                       count_NOP <= count_NOP + 1;
                     end
                  end
                  else begin
                  state <= PRECHARGE;
                  DATA_get <= 0; 
                  count_wrt <= 0;
                  end
                end

                if (count >= 9 && count <= 17) begin
                  flag_DQSadd <= 1;
                end

              end
              
            //end

            //count_wrt <= count_wrt + 1;
          end

        ATOMICREAD: begin
          if(count_rd == ) begin
              {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
              count_rd <= count_rd + 1; 
          end
          else if(count_rd == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0101;
            A[9:0] <= rd_addr[9:0];  //colmn address check this out
            A[10]<=1'b0;                           
            BA <= rd_addr[25:23];
          end
          else if(count_rd == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
          end
          else if(count_rd == ) begin
            listen <= 1'b1;                   
            DATA_get <= 1'b1;
          end
          else if(count_rd == ) begin
            listen <= 1'b0;                            
            DATA_get <= 1'b0;
            ts_con <= 0;
            Pointer <= 0;
          end
          else if(count_rd == ) begin
            RETURN_data_temp <= RETURN_data;
            DATA_get <= 1'b0;
            RETURN_put <= 1'b1;
            RETURN_address <= rd_addr; 
                
            if (OP == NOT) begin
              RETURN_data_temp1 <= ~(RETURN_data_temp);
            end
            else if (OP == NAND) begin
              RETURN_data_temp1 <= ~(DATA_data_out & RETURN_data_temp);
            end
            else if (OP == NOR) begin
              RETURN_data_temp1 <= ~(DATA_data_out | RETURN_data_temp);
            end
            else if (OP == XOR) begin
              RETURN_data_temp1 <= DATA_data_out ^ RETURN_data_temp;
            end
            else if (OP == ADD) begin
              RETURN_data_temp1 <= DATA_data_out + RETURN_data_temp;  
            end
            else if (OP == FLIP) begin
              for (int i = 0; i < 16; i++) begin
                  for (int j = 15; j >= 0; j--) begin
                    RETURN_data_temp1[i] <= RETURN_data_temp[j];
                  end
              end    
            end
            else if (OP == SRA) begin
              RETURN_data_temp1 <= {RETURN_data_temp[15:1],1'b0};
            end
            else if (OP == SLA) begin
              RETURN_data_temp1 <= {1'b0,RETURN_data_temp[14:0]};
            end
            else begin
              RETURN_data_temp1 <= RETURN_data_temp;
            end
          end
          else if(count_rd == ) begin
            RETURN_put <= 1'b0;
            {cs_bar,ras_bar,cas_bar,we_bar} <= 4'b0111;
          end
          else if(count_rd == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar}<=4'b0100;   //issue write command
            A[9:0] <= rd_addr[9:0];                  
            A[10] <= 0;                            
            BA <= rd_addr[25:23];
            ts_con <= 1'b1;
            flag_automic <= 1;
          end
          else if(count_rd == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar}<=4'b0111;
          end
          else if(count_rd == ) begin
            flag_DQSend <= 1;
            DATA_data_out_temp <= RETURN_data_temp1;
          end
          else if(count_rd == ) begin
            DM_flag <= 1'b1;
          end
          else if(count_rd == ) begin
            DM_flag <= 1'b0;
          end
          else if(count_rd == ) begin
            count_rd <= 0;
            flag_automic <= 0;
            state <= PRECHARGE;
          end
          else begin
            state <= ATOMICREAD;
            count_rd <= count_rd + 1;
          end 

          if(count_rd >= && >= count_rd) begin
            flag_DQSadd <= 1;
          end
          else begin
            flag_DQSadd <= 0;
          end  
          
          end

        ATOMICWRITE: begin
          if(count_wrt == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0111;
            count_wrt <= count_wrt + 1; 
          end
          else if(count_wrt == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar} <= 4'b0101;
            A[9:0] <= wrt_addr[9:0];  //colmn address check this out
            A[10]<=1'b0;                           
            BA <= wrt_addr[25:23];
          end
          else if(count_wrt == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar}<=4'b0111;
          end
          else if(count_wrt == ) begin
            listen <= 1'b1;                    
            DATA_get <= 1'b1;
          end
          else if(count_wrt == ) begin
            listen <= 1'b0;                     
            DATA_get <= 1'b0;
          end
          else if(count_wrt == ) begin
            RETURN_data_temp <= RETURN_data;
            Pointer <= 0;
            DATA_get <= 1'b0;
                
            if (OP == NOT) begin
              RETURN_data_temp1 <= ~(RETURN_data_temp);
            end
            else if (OP == NAND) begin
              RETURN_data_temp1 <= ~(DATA_data_out & RETURN_data_temp);
            end
            else if (OP == NOR) begin
              RETURN_data_temp1 <= ~(DATA_data_out | RETURN_data_temp);
            end
            else if (OP == XOR) begin
              RETURN_data_temp1 <= DATA_data_out ^ RETURN_data_temp;
            end
            else if (OP == ADD) begin
              RETURN_data_temp1 <= DATA_data_out + RETURN_data_temp;  
            end
            else if (OP == FLIP) begin
              for (int i = 0; i < 16; i++) begin
                  for (int j = 15; j >= 0; j--) begin
                    RETURN_data_temp1[i] <= RETURN_data_temp[j];
                  end
              end  
            end
            else if (OP == SRA) begin
              RETURN_data_temp1 <= {RETURN_data_temp[15:1],1'b0};
            end
            else if (OP == SLA) begin
              RETURN_data_temp1 <= {1'b0,RETURN_data_temp[14:0]};
            end
            else begin
              RETURN_data_temp1 <= RETURN_data_temp;
            end
          end
          else if(count_wrt == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar}<=4'b0100;   //issue write command
            A[9:0] <= rd_addr[9:0];                  
            A[10] <= 0;                            
            BA <= rd_addr[25:23];
            ts_con <= 1'b1;
          end
          else if(count_wrt == ) begin
            {cs_bar, ras_bar, cas_bar, we_bar}<=4'b0111;
          end
          else if(count_wrt == ) begin
            flag_DQSend <= 1;
            DATA_data_out_temp <= RETURN_data_temp1;
          end
          else if(count_wrt == ) begin
            DM_flag <= 1'b1;
          end
          else if(count_wrt == ) begin
            DM_flag <= 1'b0;
          end
          else if(count_wrt == ) begin
            ts_con<=1'b0;
          end
          else if(count_wrt == ) begin
            count_wrt <= 0;
            flag_automic <= 0;
            state <= PRECHARGE;
          end
          else begin
            state <= ATOMICWRITE;
            count_wrt <= count_wrt + 1;
          end

          if(count_wrt >= && >= count_wrt) begin
            flag_DQSadd <= 1;
          end
          else begin
            flag_DQSadd <= 0;
          end  

          end 

        default : state <= IDLE;
      endcase

     end
end

always @(posedge clk) begin

  if(reset) begin
    DQS_out <= 0;
  end
  else begin
    if(flag_DQSstart) begin
      DQS_out[0] <= ~DQS_out[0];
      DQS_out[1] <= ~DQS_out[1];
    end
    else if(flag_DQSadd) begin
      DQS_out[0] <= ~DQS_out[0];
      DQS_out[1] <= ~DQS_out[1];
    end
    else if(flag_DQSend) begin
      DQS_out <= 0;
    end
  end

end

ddr3_ring_buffer8 ring_buffer(RETURN_data, listen, DQS_in[0], Pointer[2:0], DQ_in, reset);


always @(negedge clk)
  if (reset) begin
    DQ_out <= 0;
  end
  else begin
    if(flag_automic) begin
      DQ_out <= DATA_data_out_temp;
    end
    else begin
      DQ_out <= DATA_data_out;
    end
    if(DM_flag)
        DM <= 2'b00;
    else
        DM <= 2'b11;	
  end
 
endmodule // ddr3_controller
