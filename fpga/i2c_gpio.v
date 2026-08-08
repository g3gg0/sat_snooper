//   ==================================================================
//   >>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
//   ------------------------------------------------------------------
//   Copyright (c) 2013 by Lattice Semiconductor Corporation
//   ALL RIGHTS RESERVED 
//   ------------------------------------------------------------------
//
//   Permission:
//
//      Lattice SG Pte. Ltd. grants permission to use this code
//      pursuant to the terms of the Lattice Reference Design License Agreement. 
//
//
//   Disclaimer:
//
//      This VHDL or Verilog source code is intended as a design reference
//      which illustrates how these types of functions can be implemented.
//      It is the user's responsibility to verify their design for
//      consistency and functionality through the use of formal
//      verification methods.  Lattice provides no warranty
//      regarding the use or functionality of this code.
//
//   --------------------------------------------------------------------
//
//                  Lattice SG Pte. Ltd.
//                  101 Thomson Road, United Square #07-02 
//                  Singapore 307591
//
//
//                  TEL: 1-800-Lattice (USA and Canada)
//                       +65-6631-2000 (Singapore)
//                       +1-503-268-8001 (other locations)
//
//                  web: http://www.latticesemi.com/
//                  email: techsupport@latticesemi.com
//
//   --------------------------------------------------------------------
//
//  Name:  i2c_gpio.v   
// 
//  Description: i2c gpio top level module
// --------------------------------------------------------------------
// Code Revision History : 
// --------------------------------------------------------------------
// Ver: | Author      |Mod. Date  |Changes Made:
// V1.0 | Akhilesh    |13/03/12   |Initial Version
// V2.0 | SHossner    |19-Sep-13  |Disabled clock stretching support per PCN#10A-13
// --------------------------------------------------------------------

`include "efb_define_def.v"
//`define INTQ_OPENDRAIN 0
`timescale 1 ns / 1 ns



module i2c_gpio   
					   (       
					    inout        SCL,
						inout        SDA,
            output reg   Enable,
						input[7:0]  Status,
						input        RST_N,
						input        CLK,
						input[31:0]  fpga_clock_cycles
                        
                        
                        );
	  
	  
 /***********************************************************************
 *                                                                     *
 * WISHBONE INTERFACE SIGNAL                                           *
 *                                                                     *
 ***********************************************************************/
reg  [7:0] wb_dat_i ;
reg        wb_stb_i ;
wire       wb_cyc_i =  wb_stb_i ;
reg  [7:0] wb_adr_i ;
reg        wb_we_i  ;
wire [7:0] wb_dat_o ;
wire       wb_ack_o ;



/***********************************************************************
 *                                                                     *
 * Data Read and Write Register                                        *
 *                                                                     *
 ***********************************************************************/
 
 
reg [7:0] data0;
reg [7:0] data1;
reg [7:0] data2;
reg [7:0] data3;
reg [7:0] temp0,temp1,temp2,temp3;
reg [7:0] n_temp0,n_temp1,n_temp2,n_temp3;
integer j;

 
 /***********************************************************************
 *                                                                      *
 * EFB Module Instanitiation                                            *
 *                                                                      *
 ***********************************************************************/
		   
wire rst_p = ~RST_N ;
efb UUT  (  .wb_clk_i (CLK      ),
            .wb_rst_i (rst_p    ),
            .wb_dat_i (wb_dat_i ),
            .wb_stb_i (wb_stb_i ),
            .wb_cyc_i (wb_cyc_i ),
            .wb_adr_i (wb_adr_i ),
            .wb_we_i  (wb_we_i  ),
            .wb_dat_o (wb_dat_o ), 
            .wb_ack_o (wb_ack_o ),      
            .i2c1_scl (SCL      ),
            .i2c1_sda (SDA      ) );
   
/***********************************************************************
 *                                                                     *
 * Signal & wire Declartion                                            *
 *                                                                     *
 ***********************************************************************/

reg        reg_rdy  , reg_rdy_del;
reg        dat_rdy  , dat_rdy_del;
reg        efb_flag , n_efb_flag ;

reg [7:0] n_dat_count , dat_count ; 

wire [7:0] i2c_cmd  ;
wire [7:0] reg_addr  ;
wire       cmd_rdy  ;

reg  [7:0] n_wb_dat_i ;
reg        n_wb_stb_i ;
reg  [7:0] n_wb_adr_i ;
reg        n_wb_we_i  ;

reg [7:0]   c_state ,n_state;
reg         n_count_en , count_en;




//////////////////////////////////////////////
//                                          //
//    I2C Data & Command Storing            //
//                                          //
//////////////////////////////////////////////   
		

always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N) begin
       reg_rdy     <= 1'b0 ;
	   reg_rdy_del <= 1'b0 ;
	 end  
	 else begin  
       reg_rdy      <= ((c_state == `state11) && wb_ack_o && efb_flag );
	   reg_rdy_del  <= reg_rdy ;
	 end   
   end 	   

always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N) begin
       dat_rdy     <= 1'b0 ;
	   dat_rdy_del <= 1'b0 ;
	 end  
	 else begin  
       dat_rdy      <= ((c_state == `state14) && wb_ack_o && efb_flag );
	   dat_rdy_del  <= dat_rdy ;
	 end   
   end 	   
   
assign  cmd_rdy  =  (n_state == `state9 );   
assign  i2c_cmd  = temp1 ;
assign  reg_addr = temp2 ;

   
always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N) begin 
        data0 <= 8'h00000000;
        data1 <= 8'h00000000;
        data2 <= 8'h00000000;
        data3 <= 8'h00000000;
    end
   // Add your logic here for data[0-7] registers
    else begin 
		case(i2c_cmd)
			`Status_Read: 		data0 <= Status; // Read GPIO Input 
			`FPGA_CLK_Read:  	
            begin 
                data3 <= fpga_clock_cycles[7:0]; 
                data2 <= fpga_clock_cycles[15:8];
                data1 <= fpga_clock_cycles[23:16];
                data0 <= fpga_clock_cycles[31:24];
            end 
			default:			data0 <= data0;
		endcase
	end   
  end   

always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N)
      begin 
        temp0 <= 8'h00 ;
        temp1 <= 8'h00 ;
      end
    else begin 
        temp0 <= n_temp0 ;
        temp1 <= n_temp1 ; 
    end
   end   
  

always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N) 
         Enable <= 1'b0 ;      
	else if (i2c_cmd == `ENABLE)
		 Enable <= 1'b1 ;
	else if (i2c_cmd == `DISABLE)
		 Enable <= 1'b0 ;
	else 
         Enable <= Enable ;
   end 	
 
/////////////////////////////////////////////
//                                         //
//        Command Decoding Logic           //
//                                         //
/////////////////////////////////////////////   
 
wire enable_command    = ((temp1 == `ENABLE) || (temp1 == `DISABLE));
wire wide_read_command = (temp1 == `FPGA_CLK_Read);												  

wire status_command      = (temp1 == `Status_Read);
wire cmd_data =  (enable_command | status_command | wide_read_command);	

wire read_command = (i2c_cmd == `Status_Read) || (i2c_cmd == `FPGA_CLK_Read);
    
/***********************************************************************
 *                                                                     *
 *                    Main State Machine                               *
 *                                                                     *
 ***********************************************************************/

 /////////////////////////////////////////////
//                                          //
//    State Machines Sequential Block       //
//                                          //
//////////////////////////////////////////////   


always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N)
      begin 
         wb_dat_i <= 8'h00;
         wb_stb_i <= 1'b0 ;
         wb_adr_i <= 8'h00;
         wb_we_i  <= 1'b0;   
      end   
    else 
      begin 
         wb_dat_i <= #1 n_wb_dat_i;
         wb_stb_i <= #1 n_wb_stb_i;
         wb_adr_i <= #1 n_wb_adr_i;
         wb_we_i  <= #1 n_wb_we_i ;

       end 
  end 

always @ (posedge CLK or negedge RST_N)
  begin 
    if(!RST_N) begin 
      c_state  <= 10'h000;
      efb_flag <= 1'b0 ;
      count_en <= 1'b0; 
	  dat_count <= 8'd0;
    end  
    else begin  
      c_state  <= n_state   ;
      efb_flag <= n_efb_flag;
      count_en <= n_count_en;
	  dat_count <= n_dat_count ;
    end  
  end

//////////////////////////////////////////////
//                                          //
//    State Machines Combinational Block    //
//                                          //
//////////////////////////////////////////////   
  
 

always @ ( * )
  begin 
     n_efb_flag   =  1'b0 ; 
     n_state      = c_state ; 
	 n_dat_count  = dat_count ;
     n_wb_dat_i  = 8'h00;
     n_wb_stb_i  = 1'b0 ;
     n_wb_adr_i  = 8'h00;
     n_wb_we_i   = 1'b0;
     n_count_en  = 1'b0; 
     n_temp0     = temp0;
     n_temp1     = temp1;
     n_temp2     = temp2;
     n_temp3     = temp3;    
  
     case(c_state)
     `state0: begin 
           n_wb_dat_i =  8'h00;
           n_wb_stb_i =  1'b0 ;
           n_wb_adr_i =  8'h00;
           n_wb_we_i  =  1'b0;           
           n_wb_stb_i =  1'b0 ;     
           n_state =  `state1 ;             
     
    end   
    
  `state1: begin // Enable I2C Interface
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
           n_state = `state2;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `WRITE;
       n_wb_adr_i = `MICO_EFB_I2C_CR;
       n_wb_dat_i = 8'h80;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state2: begin // Clock Disable
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
           n_state = `state3;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `WRITE;
       n_wb_adr_i = `MICO_EFB_I2C_CMDR;
       n_wb_dat_i = 8'h04;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state3: begin // Wait for not BUSY
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
       if(wb_dat_o & ( `MICO_EFB_I2C_SR_BUSY) )
           n_state = c_state;
       else
           n_state = `state4;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state4: begin // Discard data 1
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
        n_temp0 = wb_dat_o; 
           n_state = `state5;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_DATA;
       n_wb_adr_i = `MICO_EFB_I2C_RXDR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state5: begin // Discard data 2
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
          n_temp0 = wb_dat_o; 
          n_state = `state7;   // keep clock stretching disabled per PCN#10A-13
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_DATA;
       n_wb_adr_i = `MICO_EFB_I2C_RXDR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
  `state7: begin // wait for data to come 
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ; 
		  n_temp1 = 8'h00; 
       if(wb_dat_o & (`MICO_EFB_I2C_SR_TRRDY)) 
	         n_state = `state8;		   
       else		   
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
	 
  `state8: begin // Store i2C Command Information
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
          n_temp1 = wb_dat_o; 
          n_state = `state9;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_DATA;
       n_wb_adr_i = `MICO_EFB_I2C_RXDR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
   end
 
 
  `state9: begin // Send ACK or NACK Based upon Command Receive & Wait for Stop `state 17
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
		  if(enable_command)                                           // Enable command
		   n_state = `state17;	
          else if (read_command)
		   n_state = `state12;
		  else                                                         // Error commands
           n_state = `state17;		

          if(wide_read_command)
            n_dat_count = 4; 
          else
            n_dat_count = 1;   
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `WRITE;
       n_wb_adr_i = `MICO_EFB_I2C_CMDR;
       n_wb_dat_i = {4'h0,~cmd_data,3'b100};    // keep clock stretching disabled per PCN#10A-13
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
 
 
  `state12: begin // Wait for TRRDY Bit 
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
       if(wb_dat_o & ( `MICO_EFB_I2C_SR_TRRDY))
	       if(read_command) // If Read Commands
            n_state = `state13;
		   else 
		    n_state =  `state14; // For write Commands
       else if (~wb_dat_o[6])
           n_state = `state2;
       else
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state13: begin // Check for read or write operation Go to State15
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
       if(wb_dat_o & ( `MICO_EFB_I2C_SR_SRW))
           n_state = `state15;
       else if (~wb_dat_o[6])
           n_state = `state2;
       else
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state14: begin // Read Data
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
          n_temp3 = wb_dat_o;		  
          n_state = `state19;
            if(dat_count != 8'h00)
                n_dat_count = dat_count - 1 ;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_DATA;
       n_wb_adr_i = `MICO_EFB_I2C_RXDR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
	 
	 
 
 
  `state15: begin // Send Data to Master
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
          n_state = `state18;
          if(dat_count != 8'h00)
            n_dat_count = dat_count - 1;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `WRITE;
       n_wb_adr_i = `MICO_EFB_I2C_TXDR;
       case(dat_count)
        0:        n_wb_dat_i = data0;
        1:        n_wb_dat_i = data0;
        2:        n_wb_dat_i = data1;
        3:        n_wb_dat_i = data2;
        4:        n_wb_dat_i = data3;
       endcase
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
  end
 
 
 
  `state16: begin // Send NACK Based upon Command Receive
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
           n_state = `state17;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `WRITE;
       n_wb_adr_i = `MICO_EFB_I2C_CMDR;
       n_wb_dat_i = 8'h0C;  //keep clock stretching disabled per PCN#10A-13
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
 
 
  `state17: begin // Wait till Stop is Send 
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
       if(~wb_dat_o[6])
           n_state = `state2;
       else
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
   end
 
   `state18: begin // Wait for TxRDY flag and send data again if required 
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
       if (( dat_count == 8'h00))	   
            n_state = `state16;		                  // Send Nack for Any More Read Request 
       else if(wb_dat_o & (`MICO_EFB_I2C_SR_TRRDY ))
           if(read_command)
     		  n_state = `state15;                   // Send Data  
		   else
              n_state = `state17;		             // Wait till Stop 
       else if (~wb_dat_o[6])// If Stop go to beginning
           n_state = `state2;
       else
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end

 
   `state19: begin // Wait for TRRDY bit 
      if (wb_ack_o && efb_flag) begin
          n_wb_dat_i = `ALL_ZERO ;
          n_wb_adr_i = `ALL_ZERO ;
          n_wb_we_i =  `LOW ;
          n_wb_stb_i = `LOW ;
          n_efb_flag = `LOW ;
          n_count_en = `LOW ;
	   if ( dat_count == 8'h00)
            n_state = `state16;		   
       else if(wb_dat_o & ( `MICO_EFB_I2C_SR_TRRDY) )
           n_state = `state14;
       else if (~wb_dat_o[6])
           n_state = `state2;
       else
           n_state = c_state;
       end
       else begin
       n_efb_flag = `HIGH ;
       n_wb_we_i =  `READ_STATUS;
       n_wb_adr_i = `MICO_EFB_I2C_SR;
       n_wb_dat_i =  0 ;
       n_wb_stb_i = `HIGH ; 
       n_state = c_state; 
    end
     end
	 
 
endcase
end
endmodule
