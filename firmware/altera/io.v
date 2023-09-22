module io(
input wire clk,
input wire rst,
//Core wires
input wire[7:0]IODIN,
input wire[5:0]IOCNT,
output reg[7:0]IODOUT,
input wire[7:0]SREG,
input wire[15:0]SP,
input wire IOW,
input wire IOR,
//INTERRUPTS
output wire IRQ_REQ,
output wire[3:0]IRQ_ADD,
//GPIO
inout wire[7:0]PB,
inout wire[7:0]PC,
inout wire[7:0]PD,
//UART
output wire uart_tx,
input wire uart_rx,
//SPI
output wire mosi,
output wire sck, 
input wire miso,
//I2C
inout sda,
inout scl,
//TIMER0
input wire t0
//DEBUG

);

`include "params.sv"

//Register map like Atmega8
localparam aSREG = 8'h3F;
localparam aSPH = 8'h3E;
localparam aSPL = 8'h3D;	
localparam aGICR = 8'h3B;
localparam aGIFR = 8'h3A;
localparam aTWCR = 8'h36;
localparam aMCUCR = 8'h35;
localparam aTWBR = 8'h00;
localparam aTWSR = 8'h01;
localparam aTWAR = 8'h02;
localparam aTWDR = 8'h03;
localparam aUBRRH = 8'h20;
localparam aUBRRL = 8'h09;
localparam aUCSRB = 8'h0A;
localparam aUCSRA = 8'h0B;
localparam aUDR = 8'h0C;
localparam aSPCR = 8'h0D;
localparam aSPSR = 8'h0E;
localparam aSPDR = 8'h0F;
localparam aPIND = 8'h10;
localparam aDDRD = 8'h11;
localparam aPORTD = 8'h12;
localparam aPINC = 8'h13;
localparam aDDRC = 8'h14;
localparam aPORTC = 8'h15;
localparam aPINB = 8'h16;
localparam aDDRB = 8'h17;
localparam aPORTB = 8'h18;
localparam aTCNT0 = 8'h32;
localparam aTCCR0 = 8'h33;
localparam aTIFR = 8'h38;
localparam aTIMSK = 8'h39;

//------------------------------INTERRUPTS-------------------------------

/*
Vector No. | Program Address | Source Interrupt Definition

1               0x000           RESET External Pin, Power-on Reset, Brown-out
2               0x001           INT0 External Interrupt Request 0
3               0x002           INT1 External Interrupt Request 1
4               0x003           TIMER2 COMP Timer/Counter2 Compare Match
5               0x004           TIMER2 OVF Timer/Counter2 Overflow
6               0x005           TIMER1 CAPT Timer/Counter1 Capture Event
7               0x006           TIMER1 COMPA Timer/Counter1 Compare Match A
8               0x007           TIMER1 COMPB Timer/Counter1 Compare Match B
9               0x008           TIMER1 OVF Timer/Counter1 Overflow
10              0x009           TIMER0 OVF Timer/Counter0 Overflow
11              0x00A           SPI, STC Serial Transfer Complete
12              0x00B           USART, RXC USART, Rx Complete
13              0x00C           USART, UDRE USART Data Register Empty
14              0x00D           USART, TXC USART, Tx Complete
15              0x00E           ADC ADC Conversion Complete
16              0x00F           EE_RDY EEPROM Ready
17              0x010           ANA_COMP Analog Comparator
18              0x011           TWI Two-wire Serial Interface
19              0x012           SPM_RDY Store Program Memory Ready
*/


`ifdef irq_enable
wire[12:0]irq_source;
reg[3:0]irq_vect;

assign IRQ_REQ = SREG[7] & (irq_source[0] | irq_source[1] | irq_source[2] | irq_source[3] | irq_source[4] | 
                    irq_source[5] | irq_source[6] | irq_source[7] | irq_source[8] | irq_source[9] | irq_source[10] |
                    irq_source[11] | irq_source[12]);

assign IRQ_ADD = irq_vect;


always@(irq_source)
begin
    if(irq_source[0] == 1) irq_vect <= 1;    
    else if(irq_source[1:0] == 2'b10) irq_vect <= 2;
    else if(irq_source[2:0] == 3'b100) irq_vect <= 3;
    else if(irq_source[3:0] == 4'b1000) irq_vect <= 4;
    else if(irq_source[4:0] == 5'b10000) irq_vect <= 5;
    else if(irq_source[5:0] == 6'b100000) irq_vect <= 6;
    else if(irq_source[6:0] == 7'b1000000) irq_vect <= 7;
    else if(irq_source[7:0] == 8'b10000000) irq_vect <= 8;
    else if(irq_source[8:0] == 9'b100000000) irq_vect <= 9;
    else if(irq_source[9:0] == 10'b1000000000) irq_vect <= 10;
    else if(irq_source[10:0] == 11'b10000000000) irq_vect <= 11;
    else if(irq_source[11:0] == 12'b100000000000) irq_vect <= 12;
    else if(irq_source[12:0] == 13'b1000000000000) irq_vect <= 13;
end

//INT0 (PD2 vector 1)
assign irq_source[0] = intf0 & int10[0] & SREG[7];

//INT1 (PD3 vector 2)
assign irq_source[1] = intf1 & int10[1] & SREG[7];

assign irq_source[2] = 0;
assign irq_source[3] = 0;
assign irq_source[4] = 0;
assign irq_source[5] = 0;
assign irq_source[6] = 0;
assign irq_source[7] = 0;

//TIMER0 Overflow
assign irq_source[8] = tov0 & toie0 & SREG[7];

assign irq_source[9] = 0;

//UART RXC(vector 11)
assign irq_source[10] = uart_rxc & uart_rxcie & SREG[7];

assign irq_source[11] = 0;
assign irq_source[12] = 0;
`endif

//----------------------------------GPIO---------------------------------

//WIRES
wire[7:0]PBI,PCI,PDI;
//GENERAL
reg[7:0]mcucr;
//GPIO Registers
reg[7:0]portb,portc,portd,ddrb,ddrc,ddrd;
reg[1:0]int10;




//RW OPERATIONS with reg
always@(posedge IOW or negedge rst)
begin
	if(!rst)
		begin
			portb <= 0;
			portc <= 0;
			portd <= 0;
			ddrb <= 0;
			ddrc <= 0;
			ddrd <= 0;
			udr_tx <= 0;
			uart_rxen <= 0;
			uart_txen <= 0;
			ubrr <= 0;
            spdr_w <= 0;
            spcr <= 0;
            spsr <= 0;
            int10 <= 0;
            mcucr <= 0;
            tccr0 <= 0;
            toie0 <= 0;
            twbr <= 0;
            twen <= 0;
            twie <= 0;
            twdr_w <= 0;
		end
	else
		begin
			case(IOCNT)
                //GENERAL
                aMCUCR: mcucr <= IODIN;
				//GPIO
				aPORTB: portb <= IODIN;
				aPORTC: portc <= IODIN;
				aPORTD: portd <= IODIN;
				aDDRB:  ddrb <= IODIN;
				aDDRC:  ddrc <= IODIN;
				aDDRD:  ddrd <= IODIN;
                aGICR:  int10[1:0] <= IODIN[7:6];
				//UART
				aUDR: udr_tx <= IODIN;
				aUCSRB: {uart_rxcie,uart_txcie,uart_udrie,uart_rxen,uart_txen} <= IODIN[7:3];
				aUBRRH:
					begin
						if(IODIN[7]);
						else ubrr[14:8] <= IODIN[6:0];
					end
				aUBRRL: ubrr[7:0] <= IODIN[7:0];
            //SPI
            aSPDR: spdr_w <= IODIN;
            aSPCR: spcr   <= IODIN;
            aSPSR: spsr[5:0] <= IODIN[5:0];
            //TIMER0
            aTIMSK: toie0 <= IODIN[0];
            aTCCR0: tccr0 <= IODIN[2:0];
            //I2C
            aTWBR: twbr <= IODIN;
            aTWCR: {twea,twista,twisto,twen,twie} <= {IODIN[6:4],IODIN[2],IODIN[0]};
            aTWDR: twdr_w <= IODIN;
				default: ;
			endcase
		end
end


always@(posedge IOR or negedge rst)
begin
	if(!rst)IODOUT <= 0;
	else	
		begin
			case(IOCNT)
				//GENERAL
				aSREG: IODOUT <= SREG;
				aSPH:  IODOUT <= SP[15:8];
				aSPL:  IODOUT <= SP[7:0];
            aMCUCR: IODOUT <= mcucr;
                //GPIO
				aDDRB: IODOUT <= ddrb;
				aDDRC: IODOUT <= ddrc;
				aDDRD: IODOUT <= ddrd;
				aPORTB: IODOUT <= portb;
				aPORTC: IODOUT <= portc;
				aPORTD: IODOUT <= portd;
				aPINB:  IODOUT <= PBI;
				aPINC:  IODOUT <= PCI;
				aPIND:  IODOUT <= PDI;
            aGICR:  IODOUT <= {int10[1:0],6'b000000};
            aGIFR:  IODOUT <= {intf1,intf0,6'b000000};
				//UART
				aUCSRA: IODOUT <= {uart_rxc,uart_txc,~utrdy,5'b00000};
				aUDR:   IODOUT <= udr_rx;
            //SPI
            aSPDR:  IODOUT <= spdr_r;
            aSPSR:  IODOUT <= {~spi_rdy,7'b0000000};
            //TIMER0
            aTCCR0: IODOUT <= {5'b00000,tccr0[2:0]};
            aTCNT0: IODOUT <= tcnt0;
            aTIFR:  IODOUT <= {7'b0000000, tov0};
            aTIMSK: IODOUT <= {7'b0000000, toie0};
            //I2C
            aTWBR: IODOUT <= twbr;
            aTWCR: IODOUT <= {~twrdy,twea,twista,twisto,1'b0,twen,~tw_ack,twie};
            aTWDR: IODOUT <= twdr_r;
				default: IODOUT <= 8'hFF;
			endcase
		end
end

//----------------------------------GPIO--------------------------------

assign PB[0] = (ddrb[0] == 1)? portb[0] : 1'bZ;
assign PB[1] = (ddrb[1] == 1)? portb[1] : 1'bZ;
assign PB[2] = (ddrb[2] == 1)? portb[2] : 1'bZ;
assign PB[3] = (ddrb[3] == 1)? portb[3] : 1'bZ;
assign PB[4] = (ddrb[4] == 1)? portb[4] : 1'bZ;
assign PB[5] = (ddrb[5] == 1)? portb[5] : 1'bZ;
assign PB[6] = (ddrb[6] == 1)? portb[6] : 1'bZ;
assign PB[7] = (ddrb[7] == 1)? portb[7] : 1'bZ;

assign PC[0] = (ddrc[0] == 1)? portc[0] : 1'bZ;
assign PC[1] = (ddrc[1] == 1)? portc[1] : 1'bZ;
assign PC[2] = (ddrc[2] == 1)? portc[2] : 1'bZ;
assign PC[3] = (ddrc[3] == 1)? portc[3] : 1'bZ;
assign PC[4] = (ddrc[4] == 1)? portc[4] : 1'bZ;
assign PC[5] = (ddrc[5] == 1)? portc[5] : 1'bZ;
assign PC[6] = (ddrc[6] == 1)? portc[6] : 1'bZ;
assign PC[7] = (ddrc[7] == 1)? portc[7] : 1'bZ;

assign PD[0] = (ddrd[0] == 1)? portd[0] : 1'bZ;
assign PD[1] = (ddrd[1] == 1)? portd[1] : 1'bZ;
assign PD[2] = (ddrd[2] == 1)? portd[2] : 1'bZ;
assign PD[3] = (ddrd[3] == 1)? portd[3] : 1'bZ;
assign PD[4] = (ddrd[4] == 1)? portd[4] : 1'bZ;
assign PD[5] = (ddrd[5] == 1)? portd[5] : 1'bZ;
assign PD[6] = (ddrd[6] == 1)? portd[6] : 1'bZ;
assign PD[7] = (ddrd[7] == 1)? portd[7] : 1'bZ;

assign PBI[0] = (ddrb[0] == 1)? 1'b0 : PB[0];
assign PBI[1] = (ddrb[1] == 1)? 1'b0 : PB[1];
assign PBI[2] = (ddrb[2] == 1)? 1'b0 : PB[2];
assign PBI[3] = (ddrb[3] == 1)? 1'b0 : PB[3];
assign PBI[4] = (ddrb[4] == 1)? 1'b0 : PB[4];
assign PBI[5] = (ddrb[5] == 1)? 1'b0 : PB[5];
assign PBI[6] = (ddrb[6] == 1)? 1'b0 : PB[6];
assign PBI[7] = (ddrb[7] == 1)? 1'b0 : PB[7];

assign PCI[0] = (ddrc[0] == 1)? 1'b0 : PC[0];
assign PCI[1] = (ddrc[1] == 1)? 1'b0 : PC[1];
assign PCI[2] = (ddrc[2] == 1)? 1'b0 : PC[2];
assign PCI[3] = (ddrc[3] == 1)? 1'b0 : PC[3];
assign PCI[4] = (ddrc[4] == 1)? 1'b0 : PC[4];
assign PCI[5] = (ddrc[5] == 1)? 1'b0 : PC[5];
assign PCI[6] = (ddrc[6] == 1)? 1'b0 : PC[6];
assign PCI[7] = (ddrc[7] == 1)? 1'b0 : PC[7];

assign PDI[0] = (ddrd[0] == 1)? 1'b0 : PD[0];
assign PDI[1] = (ddrd[1] == 1)? 1'b0 : PD[1];
assign PDI[2] = (ddrd[2] == 1)? 1'b0 : PD[2];
assign PDI[3] = (ddrd[3] == 1)? 1'b0 : PD[3];
assign PDI[4] = (ddrd[4] == 1)? 1'b0 : PD[4];
assign PDI[5] = (ddrd[5] == 1)? 1'b0 : PD[5];
assign PDI[6] = (ddrd[6] == 1)? 1'b0 : PD[6];
assign PDI[7] = (ddrd[7] == 1)? 1'b0 : PD[7];

//EXTI
reg intf0,intf1;
wire INT0_TRIG, INT1_TRIG, INT0_RST, INT1_RST;
assign INT0_TRIG = (mcucr[0])? (PDI[2] & int10[0]) : (~PDI[2] & int10[0]);
assign INT1_TRIG = (mcucr[2])? (PDI[3] & int10[1]) : (~PDI[3] & int10[1]);
assign INT0_RST = (IOCNT == aGIFR)? ~(IOW & IODIN[6]) : 1'b1;
assign INT1_RST = (IOCNT == aGIFR)? ~(IOW & IODIN[7]) : 1'b1;

always@(posedge INT0_TRIG or negedge INT0_RST)
begin
    if(!INT0_RST) intf0 <= 0;
    else intf0 <= 1;
end

always@(posedge INT1_TRIG or negedge INT1_RST)
begin
    if(!INT1_RST) intf1 <= 0;
    else intf1 <= 1;
end

//--------------------------------UART-----------------------------------
reg[7:0]udr_tx, udr_rx;
reg[14:0]ubrr, utx_counter;
reg uart_txen, uart_rxen, uart_txc, uart_rxc, uart_udrie, uart_txcie, uart_rxcie;
reg[3:0]utx_state;
reg utx, utrdy;

wire urx_clk, utx_clk;
wire utx_start;
wire utxc_rst;

assign urx_clk = uart_rxen & clk;
assign utx_clk = uart_txen & clk;
assign utx_start = (IOCNT == aUDR)? IOW : 1'b0;
assign utxc_rst = (IOCNT == aUCSRA)? ~(IODIN[6] & IOW) : 1'b1;

//TXC Flag 
always@(posedge ~utrdy or negedge utxc_rst)
begin
    if(!utxc_rst) uart_txc <= 0;
    else uart_txc <= 1;
end

//UART TX State machine	
always@(posedge utx_clk or negedge rst)
	begin
        if(!rst)
            begin
                utx <= 0;
                utrdy <= 0;
                utx_counter <= 0;
                utx_state <= 0;
            end
		 else
            begin
                if(utx_counter > 0) utx_counter <= utx_counter - 1;
                //
                case(utx_state)
                    0:
                        begin
                            utx <= 0;
                            utrdy <= 0; 
                            if(utx_start)
                                begin
                                    utrdy <= 1;
                                    utx <= 1;
                                    utx_counter <= ubrr;
                                    utx_state <= 1;
                                end
                        end
                    1: //START Bit
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[0];
                                    utx_counter <= ubrr;
                                    utx_state <= 2;
                                end
                        end
                    2: //Bit0
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[1];
                                    utx_counter <= ubrr;
                                    utx_state <= 3;
                                end
                        end
                    3: //Bit1
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[2];
                                    utx_counter <= ubrr;
                                    utx_state <= 4;
                                end
                        end
                    4: //Bit2
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[3];
                                    utx_counter <= ubrr;
                                    utx_state <= 5;
                                end
                        end
                    5: //Bit3
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[4];
                                    utx_counter <= ubrr;
                                    utx_state <= 6;
                                end
                        end
                    6: //Bit4
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[5];
                                    utx_counter <= ubrr;
                                    utx_state <= 7  ;
                                end
                        end
                    7: //Bit5
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[6];
                                    utx_counter <= ubrr;
                                    utx_state <= 8;
                                end
                        end
                    8: //Bit6
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= ~udr_tx[7];
                                    utx_counter <= ubrr;
                                    utx_state <= 9;
                                end
                        end
                    9: //Bit7
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx <= 0;
                                    utx_counter <= ubrr;
                                    utx_state <= 10;
                                end
                        end
                    10: //Stop
                        begin
                            if(utx_counter == 0)
                                begin
                                    utx_counter <= ubrr;
                                    utx_state <= 11;
                                end
                        end
                    11: //Return to IDDLE
                        begin
                            utrdy <= 0;
                            utx_state <= 0;
                        end
                endcase
            end
	end

assign uart_tx = ~utx;

//UART RX
reg urrdy;
wire urxc_rst;
assign urxc_rst = (IOCNT == aUDR)? ~IOR : 1'b1;

//RXC Flag 
always@(negedge urrdy or negedge urxc_rst)
begin
    if(!urxc_rst) uart_rxc <= 0;
    else uart_rxc <= 1;
end

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
  reg [2:0]     r_SM_Main     = 0;
   

  always @(posedge clk)
    begin
      r_Rx_Data_R <= uart_rx;
      r_Rx_Data   <= r_Rx_Data_R;
    end
   
   
  // Purpose: Control RX state machine
  always @(posedge clk)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            urrdy       <= 1'b0;
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
				urrdy       <= 1'b1;
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
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
              end
          end // case: s_RX_STOP_BIT
     
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_SM_Main <= s_IDLE;
            urrdy   <= 1'b0;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end

//----------------------------------SPI--------------------------------------
//SPCR -> SPIE : SPE : DORD : MSTR : CPOL : CPHA : SPR1 : SPR0
reg[7:0]spcr,spdr_r,spdr_w,spdr_t;
reg[5:0]spsr;
reg[4:0]spi_state;
reg[7:0]spi_delay;
reg spi_sck, spi_so, spi_rdy;

wire spi_start;
wire spi_clk;
assign spi_start = (IOCNT == aSPDR)? IOW : 1'b0;
assign spi_clk = clk & spcr[6];

always@(posedge spi_clk or negedge rst)
begin
    if(!rst)
        begin
            spi_state <= 0;
            spi_delay <= 0;
            spi_sck <= 0;
            spi_so <= 0;
            spi_rdy <= 0;
            spdr_r <= 0;
            spdr_t <= 0;
        end
    else
        begin
            if(spi_delay > 0) spi_delay <= spi_delay - 1;
            case(spi_state)
                0: //IDDLE
                    begin
                        spi_sck <= 0;
                        spi_so <= 0;
                        spi_rdy <= 0;
                        spi_state <= 0;
                        if(spi_start)
                            begin
                                spi_state <= 1;
                                spi_rdy <= 1;
                                spi_delay <= {spsr[5:0],spcr[1:0]};
                            end
                    end
                1: //Load First bit _
                    begin
						spi_so <= spdr_w[7];
						if(spi_delay == 0)
							begin
								spi_state <= 2;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end
                    end
                2: //SCK HIGH 0
                    begin
						spdr_t[7] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 3;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                3: //SCK LOW 0
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[6];
						if(spi_delay == 0)
							begin
								spi_state <= 4;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end 
                4: //SCK HIGH 1
                    begin
						spdr_t[6] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 5;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                5: //SCK LOW 1
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[5];
						if(spi_delay == 0)
							begin
								spi_state <= 6;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end                    
                6: //SCK HIGH 2
                    begin
						spdr_t[5] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 7;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                7: //SCK LOW 2
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[4];
						if(spi_delay == 0)
							begin
								spi_state <= 8;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end
                8: //SCK HIGH 3
                    begin
						spdr_t[4] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 9;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                9: //SCK LOW 3
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[3];
						if(spi_delay == 0)
							begin
								spi_state <= 10;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end 
                10: //SCK HIGH 4
                    begin
						spdr_t[3] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 11;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                11: //SCK LOW 4
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[2];
						if(spi_delay == 0)
							begin
								spi_state <= 12;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end 
                12: //SCK HIGH 5
                    begin
						spdr_t[2] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 13;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                13: //SCK LOW 5
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[1];
						if(spi_delay == 0)
							begin
								spi_state <= 14;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end
                14: //SCK HIGH 6
                    begin
						spdr_t[1] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 15;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                15: //SCK LOW 6
                    begin
						spi_sck <= 0;
						spi_so <= spdr_w[0];
						if(spi_delay == 0)
							begin
								spi_state <= 16;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end 
                16: //SCK HIGH 7
                    begin
						spdr_t[0] <= miso;
						spi_sck <= 1;
						if(spi_delay == 0)
							begin
								spi_state <= 17;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end						
                    end
                17: //SCK LOW 7
                    begin
						spi_sck <= 0;
						if(spi_delay == 0)
							begin
								spi_state <= 18;
								spi_delay <= {spsr[5:0],spcr[1:0]};
							end			 
                    end                                                                                                          
                18: //END
                    begin
						spdr_r <= spdr_t;
                        spi_state <= 0;
                        spi_rdy <= 0;
                        spi_so <= 0;
                    end
            endcase
        end
end

assign mosi = (spcr[3])? ~spi_so : spi_so;
assign sck = (spcr[3])? ~spi_sck : spi_sck;

//----------------------------------TIMER0------------------------------------
reg[7:0]tcnt0;
reg[2:0]tccr0;
reg[9:0]t0_del, t0_psc;
reg toie0,tov0;

wire tc0_clk;
wire tc0_extclk;
wire preload;
wire tov0_rst;
assign tc0_clk = (tccr0[2:1] == 3)? tc0_extclk : (clk & (tccr0[0]|tccr0[1]|tccr0[2]));
assign tc0_extclk = (tccr0[0])? t0 : ~t0;
assign preload = (IOCNT == aTCNT0)? IOW : 0;
assign tov0_rst = (IOCNT == aTIFR)? ~(IOW & IODIN[0]) : 1;

always@(negedge tcnt0[7] or negedge (tov0_rst))
begin
    if(!(tov0_rst)) tov0 <= 0;
    else tov0 <= 1;
end

always@(tccr0)
begin
	case(tccr0)
		2: t0_psc <= 7;
		3: t0_psc <= 63;
		4: t0_psc <= 255;
		5: t0_psc <= 1023;
		default: t0_psc <= 0;
	endcase
end

always@(posedge tc0_clk or negedge rst)
begin
	if(!rst)
		begin
			tcnt0 <= 0;
		end
	else
		begin
			if(t0_del > 0) t0_del <= t0_del - 1;
			if(preload) 
				begin
					tcnt0 <= IODIN;
					t0_del <= t0_psc;
				end
			else
				begin
					if(t0_del == 0) 
						begin
							tcnt0 <= tcnt0 + 1;
							t0_del <= t0_psc;
						end
				end
		end
end


//---------------------------------I2C------------------------------------
reg twen,twie,twea,twista,twisto;
reg[7:0]twbr;
reg[9:0]tw_delay;
reg[7:0]twdr_w,twdr_r,twdr_t;
reg twdr_full;
reg scl_dir, sda_dir;
reg twrdy,tw_ack;
reg[5:0]tw_state;


wire twdr_clk,twdr_rst;
assign twdr_clk = (IOCNT == aTWDR)? IOW : 1'b0;
assign twdr_rst = (tw_state == 8)? 0 : 1;
wire tw_start;
assign tw_start = (IOCNT == aTWCR)? (IOW & IODIN[7]) : 1'b0;
assign sda = (sda_dir)? 1'b0 : 1'bz;
assign scl = (scl_dir)? 1'b0 : 1'bz;

always@(posedge twdr_clk or negedge twdr_rst)
begin
    if(!twdr_rst) twdr_full <= 0;
    else twdr_full <= 1;
end

always@(posedge clk or negedge rst)
begin
    if(!rst)
        begin
            tw_state <= 0;
            scl_dir <= 0;
            sda_dir <= 0;
            twrdy <= 0;
            twdr_r <= 0;
            twdr_t <= 0;
            tw_ack <= 0;
            tw_delay <= 0;
        end
    else
        begin
           if(tw_delay > 0) tw_delay <= tw_delay - 1;
            case(tw_state)
                0: //IDDLE
                    begin
                        tw_state <= 0;
                        twrdy <= 0;
                        if(tw_start)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                case({twista,twisto,twdr_full})
									3'b100: tw_state <= 1; //I2C START
									3'b010: tw_state <= 4; //I2C STOP
									3'b001: tw_state <= 7; //I2C WRITE BYTE
									3'b000: tw_state <= 26; //I2C READ BYTE
									default: tw_state <= 0;
								endcase
								/*
                                if(twista) tw_state <= 1; //I2C START
                                else if(twisto) tw_state <= 4; //I2C STOP 
                                else if(twdr_full) tw_state <= 7; //I2C WRITE BYTE
                                else tw_state <= 26; //I2C READ BYTE
                                */
                            end
                    end
                1: //START
                    begin
                        twrdy <= 1;
                        sda_dir <= 0;
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 2;
                            end
                    end
                2:
                    begin
                        sda_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 3; 
                            end
                    end
                3:
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 63; 
                            end
                    end
                4: //STOP
                    begin
                        twrdy <= 1;
                        sda_dir <= 1;
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 5;
                            end
                    end
                5: 
                    begin
                        sda_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 6;
                            end
                    end
                6: 
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 63;
                            end
                    end
                7: //WRITE BYTE
                    begin
                        twrdy <= 1;
                        sda_dir <= ~twdr_w[7];
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 8;
                            end
                    end
                8: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 9;
                            end
                    end
                9: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[6];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 10;
                            end
                    end
                10: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 11;
                            end
                    end
                11: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[5];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 12;
                            end
                    end
                12: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 13;
                            end
                    end
                13: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[4];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 14;
                            end
                    end
                14: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 15;
                            end
                    end
                15: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[3];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 16;
                            end
                    end
                16: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 17;
                            end
                    end
                17: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[2];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 18;
                            end
                    end
                18: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 19;
                            end
                    end
                19: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[1];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 20;
                            end
                    end
                20: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 21;
                            end
                    end
                21: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= ~twdr_w[0];
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 22;
                            end
                    end
                22: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 23;
                            end
                    end
                23: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 24;
                            end
                    end
                24: //SCK HIGH
                    begin
                        scl_dir <= 0;   
                        if(tw_delay == {twbr,1'b1}) tw_ack <= sda;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 25;
                            end
                    end
                25: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 63;
                            end
                    end
                26: //READ BYTE
                    begin
                        twrdy <= 1;
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 27;
                            end
                    end
                27: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        sda_dir <= 0;               
                        if(tw_delay == {twbr,1'b1}) twdr_t[7] <= sda;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 28;
                            end
                    end
                28: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 29;
                            end
                    end
                29: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[6] <= sda;
                        if(tw_delay == 0)                 
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 30;
                            end
                    end
                30: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 31;
                            end
                    end
                31: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[5] <= sda;
                        if(tw_delay == 0)                       
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 32;
                            end
                    end
                32: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 33;
                            end
                    end
                33: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[4] <= sda;
                        if(tw_delay == 0)                    
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 34;
                            end
                    end
                34: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 35;
                            end
                    end
                35: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[3] <= sda;
                        if(tw_delay == 0)                      
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 36;
                            end
                    end
                36: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 37;
                            end
                    end
                37: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[2] <= sda;
                        if(tw_delay == 0)                      
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 38;
                            end
                    end
                38: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 39;
                            end
                    end
                39: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[1] <= sda;
                        if(tw_delay == 0)                      
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 40;
                            end
                    end
                40: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 41;
                            end
                    end
                41: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == {twbr,1'b1}) twdr_t[0] <= sda;
                        if(tw_delay == 0)                       
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 42;
                            end
                    end
                42: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == {twbr,1'b1}) sda_dir <= twea;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 43;
                            end
                    end
                43: //SCK HIGH
                    begin
                        scl_dir <= 0;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 44;
                            end
                    end
                44: //SCK LOW
                    begin
                        scl_dir <= 1;
                        if(tw_delay == 0)
                            begin
                                tw_delay <= {twbr[7:0], 2'b11};
                                tw_state <= 45;
                            end
                    end
                45: 
                    begin
                        twdr_r <= twdr_t;
                        tw_state <= 63;
                    end
                63: //EXIT
                    begin
                        twrdy <= 0;
                        tw_state <= 0;                               
                    end
            endcase
        end
end

endmodule
