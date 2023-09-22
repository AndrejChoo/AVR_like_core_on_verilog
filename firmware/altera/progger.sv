module progger(
	input wire clk,
	input wire rst,
	input wire rx,
	output wire[15:0]DOUT,
	output wire[ROM_ADD_WIDTH:0]PADD,
	output wire wren,
	output wire clock
);

`include "params.sv"

//UART RX state machine
localparam INCLOCK = 50000000;
localparam BAUDE = 115200;
localparam ubrr = INCLOCK / BAUDE;

reg[7:0]udr_rx;

 parameter s_IDLE         = 3'b000;
  parameter s_RX_START_BIT = 3'b001;
  parameter s_RX_DATA_BITS = 3'b010;
  parameter s_RX_STOP_BIT  = 3'b011;
  parameter s_CLEANUP      = 3'b100;
   
  reg           r_Rx_Data_R = 1'b1;
  reg           r_Rx_Data   = 1'b1;
   
  reg [14:0]    r_Clock_Count = 0;
  reg [2:0]     r_Bit_Index   = 0; 
  reg [7:0]     r_Rx_Byte     = 0;
  reg           r_Rx_DV       = 0;
  reg [2:0]     r_SM_Main     = 0;
   

  always @(posedge clk)
    begin
      r_Rx_Data_R <= rx;
      r_Rx_Data   <= r_Rx_Data_R;
    end
   
   
  // Purpose: Control RX state machine
  always @(posedge clk)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            r_Rx_DV       <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;
             
            if (r_Rx_Data == 1'b0)          // Start bit detected
              r_SM_Main <= s_RX_START_BIT;
            else
              r_SM_Main <= s_IDLE;
          end
         
        // Check middle of start bit to make sure it's still low
        s_RX_START_BIT :
          begin
            if (r_Clock_Count == (ubrr-1)/2)
              begin
                if (r_Rx_Data == 1'b0)
                  begin
                    r_Clock_Count <= 0;  // reset counter, found the middle
                    r_SM_Main     <= s_RX_DATA_BITS;
                  end
                else
                  r_SM_Main <= s_IDLE;
              end
            else
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_START_BIT;
              end
          end // case: s_RX_START_BIT
         
         
        // Wait CLKS_PER_BIT-1 clock cycles to sample serial data
        s_RX_DATA_BITS :
          begin
            if (r_Clock_Count < ubrr-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count          <= 0;
                r_Rx_Byte[r_Bit_Index] <= r_Rx_Data;
                 
                // Check if we have received all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_RX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_RX_STOP_BIT;
                  end
              end
          end // case: s_RX_DATA_BITS
     
     
        // Receive Stop bit.  Stop bit = 1
        s_RX_STOP_BIT :
          begin
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
				udr_rx <= r_Rx_Byte;
            if (r_Clock_Count < ubrr-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_STOP_BIT;
              end
            else
              begin
                r_Rx_DV       <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
              end
          end // case: s_RX_STOP_BIT
     
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_SM_Main <= s_IDLE;
            r_Rx_DV   <= 1'b0;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end



//Flash state machine
reg[15:0]pdata;
reg[ROM_ADD_WIDTH:0]padd;
reg we, pclock;
reg[3:0]f_state;
reg hlbyte;

wire f_start;
assign f_start = (r_SM_Main == s_CLEANUP)? 1'b1 : 1'b0;

always@(posedge clk or posedge rst)
begin
	if(rst)
		begin
			f_state <= 0;
			pdata <= 0;
			padd <= 0;
			we <= 0;
			pclock <= 0;
			hlbyte <= 0;
		end
	else
		begin
			case(f_state)
				0:
					begin
						if(f_start)f_state <= 1;					
					end
				1:
					begin
						if(hlbyte)
							begin
								pdata[7:0] <= udr_rx;
								hlbyte <= 0;
								we <= 1;
								f_state <= 2;
							end
						else
							begin
								pdata[15:8] <= udr_rx;
								hlbyte <= 1;
								f_state <= 0;
							end
						end
				2:
					begin
						pclock <= 1;
						f_state <= 3;
					end
				3:
					begin
						f_state <= 4;
					end
				4:
					begin
						pclock <= 0;
						f_state <= 5;
					end
				5:
					begin
						we <= 0;
						padd <= padd + 1;
						f_state <= 0;
					end					
			endcase
		end
end

assign clock = ~pclock;
assign wren = we;
assign PADD = padd;
assign DOUT = pdata;

endmodule
