//`define ind7seg

module avr_new(
	input wire clk,
	input wire rst,
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
	input wire t0,
	//DEBUG
`ifdef ind7seg
	output wire[7:0]RAZR,
	output wire[7:0]SEG,
`endif
	input wire btn
);

//WIRES
wire[7:0]SREG;
wire[15:0]SP;
wire[15:0]PCNT;
wire[6:0]ONUM;
wire[5:0]IOCNT;
wire[7:0]IODIN, IODOUT;
wire[3:0]IRQ_ADD;
wire IOW,IOR;
wire IRQ_REQ;
//Clock
/*
	div[?]: 0 - 25MHz, 1 - 12,5MHz, 2 - 6,25MHz, 3 - 3,125MHz, 4 - 1,5625MHz, 5 - 781,25KHz, 6 - 390,625KHz, 7 - 195,3125KHz
*/
wire hclk;
assign hclk = div[0] & btn;
wire uart_rx_io, uart_rx_prog;
assign uart_rx_io = (rst)? uart_rx : 1'b1;
assign uart_rx_prog = (rst)? 1'b1 : uart_rx;

wire[15:0]PROGDATA,PROGADD;
wire PROGCLK, PROGWE;


//IN CLOCK DIVIDER /2 /4
reg[22:0]div;
always@(posedge clk) div <= div + 1;

//Interconnections
core cpu0(.clk(hclk),.rst(rst),.SREG(SREG),.IOCNT(IOCNT),.IODOUT(IODOUT),.IODIN(IODIN),.IOW(IOW),.IOR(IOR),.SP(SP),//
			 .IRQ_REQ(IRQ_REQ),.IRQ_ADD(IRQ_ADD),
			 .PROGDI(PROGDATA),.PROGADD(PROGADD),.PROG_CLK(PROGCLK),.PROG_WE(PROGWE),
			 //DEBUG
			 .PCNT(PCNT),.ONUM(ONUM));
			 
progger mpr(.clk(clk),.rst(rst),.rx(uart_rx_prog),.DOUT(PROGDATA),.PADD(PROGADD),.wren(PROGWE),.clock(PROGCLK));
				

io mio0(.clk(hclk),.rst(rst),.SREG(SREG),.IOCNT(IOCNT),.IODIN(IODOUT),.IODOUT(IODIN),.IOW(IOW),.IOR(IOR),
			.PB(PB),.PC(PC),.PD(PD),.SP(SP),.uart_rx(uart_rx_io),.uart_tx(uart_tx),.mosi(mosi),.miso(miso),.sck(sck),
			.sda(sda),.scl(scl),.t0(t0),
			.IRQ_REQ(IRQ_REQ),.IRQ_ADD(IRQ_ADD));
`ifdef ind7seg			
din7seg m7s(.clk(clk),.RAZR(RAZR),.SEG(SEG),.I0(PCNT[3:0]),.I1(PCNT[7:4]),.I2(PCNT[11:8]),.I3(PCNT[15:12]),
				.I6(ONUM[3:0]),.I7(ONUM[6:4]),.I4(SREG[3:0]),.I5(SREG[7:4]));
`endif			

endmodule

