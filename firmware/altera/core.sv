module core(
input wire clk,
input wire rst,
//IO
output wire[7:0]IODOUT,
input wire[7:0]IODIN,
output wire[5:0]IOCNT,
output wire IOW,
output wire IOR,
output wire[7:0]SREG,
output wire[15:0]SP,
//INTERRUPTS
input wire IRQ_REQ,
input wire[5:0]IRQ_ADD,
//PROGGER
input wire[15:0]PROGDI,
input wire[ROM_ADD_WIDTH:0]PROGADD,
input wire PROG_CLK,
input wire PROG_WE,
//DEBUG
output wire[15:0]PCNT,
output wire[6:0]ONUM
);

`include "params.sv"

reg state,state1;
//Registr decodera opcodov
reg[6:0] o_num;
//
reg[3:0]dop;
reg[2:0]obr;
//Registr ALU
reg[7:0]RALU;
reg[15:0]RALU16;
//Rabochie registry
reg[7:0]r[0:31]; //r[0] - r[31];
reg[7:0]sreg;
//Operands
reg[7:0]operand, operand1, CONST; 
reg[15:0]operand3;
reg[4:0]Rd,Rr;
reg[ROM_ADD_WIDTH:0]PC;
reg[15:0]sp;
//IO_REGS
reg iowr, iord;
reg[5:0]iocnt;
reg[7:0]iodout;
//RAM REGS
reg[15:0] ram_add0, ram_add1;
reg[7:0] ram_din0, ram_din1;
reg ram_we0, ram_we1;
reg ram_clk0, ram_clk1;
reg ram_ck0, ram_ck1;


//Wires
wire hclk;
wire[15:0]opcode0,opcode1;
wire[ROM_ADD_WIDTH:0]rom_add1;
wire rrd,rrd1,romwe;
wire[7:0] RAM_DOUT0, RAM_DOUT1;
//Two words command
wire comm_width;

//Assignations
assign hclk = clk;
assign rom_add1 = (rst)? (PC + 1) : PROGADD;
assign rrd = (state == 0)? ~(hclk & rst) : 1'b1;
assign rrd1 = (rst)? rrd : PROG_CLK;
assign romwe = (rst)? 1'b0 : PROG_WE;

assign IOCNT = iocnt;
assign IODOUT = iodout;
assign SREG = sreg;
assign SP = sp;
assign IOW = iowr;
assign IOR = iord;

assign comm_width = (
`ifdef c_jmp			{opcode1[15:9],opcode1[3:0]} == 11'h4AC || `endif
`ifdef c_call			{opcode1[15:9],opcode1[3:0]} == 11'h4AE || `endif
							{opcode1[15:9],opcode1[3:0]} == 11'h480 ||
							{opcode1[15:9],opcode1[3:0]} == 11'h490 )? 1 : 0;

							
//DEBUG 
assign PCNT = PC;
assign ONUM = o_num;


//Integer
integer i;

always@(posedge ram_clk0 or posedge hclk)
begin
	if(hclk) ram_ck0 <= 0;
	else ram_ck0 <= 1;
end

always@(posedge ram_clk1 or posedge hclk)
begin
	if(hclk) ram_ck1 <= 0;
	else ram_ck1 <= 1;
end

/*
ROM mr0(.q_a({opcode0[7:0],opcode0[15:8]}),.q_b({opcode1[7:0],opcode1[15:8]}),.address_a(PC),
			.address_b(rom_add1),.clock_a(rrd),.clock_b(rrd));
*/
SROM mr1(.address_a(PC),.address_b(rom_add1),.clock_a(rrd),.clock_b(rrd1),.wren_a(1'b0),.wren_b(romwe),
			.data_a(16'hFF),.data_b(PROGDI),.q_a({opcode0[7:0],opcode0[15:8]}),.q_b({opcode1[7:0],opcode1[15:8]}));


RAM ms0(.address_a(ram_add0[RAM_ADD_WIDTH:0]),.address_b(ram_add1[RAM_ADD_WIDTH:0]),.clock_a(~ram_ck0),.clock_b(~ram_ck1),
			.wren_a(ram_we0),.wren_b(ram_we1),.data_a(ram_din0),.data_b(ram_din1),.q_a(RAM_DOUT0),.q_b(RAM_DOUT1));

			
//Logika mashinnogo tcikla
always@(negedge hclk or negedge rst)
begin
	if(!rst) state <= 0;
	else state <= state + 1;
end

always@(posedge hclk or negedge rst)
begin
	if(!rst) state1 <= 0;
	else state1 <= state1 + 1;
end

always@(negedge hclk or negedge rst)
begin
	if(!rst)
		begin
			PC <= 0;
			sreg <= 0;
			iowr <= 0;
			iord <= 0;
			iodout <= 0;
			sp <= 0;
			dop <= 0;
			obr <= 0;
			ram_clk0 <= 0;
			ram_clk1 <= 0;
			ram_din0 <= 0;
			ram_din1 <= 0;
			ram_we0 <= 0; 
			ram_we1 <= 0;
			ram_add0 <= 0; 
			ram_add1 <= 1;
			for(i = 0; i < 32; i = i + 1) r[i] <= 0;
         Rd <= 0;
         operand1 <= 0;
         operand3 <= 0;
`ifdef debug
			//DEBUG
			OPCNT <= 0;
`endif
		end
	else
		begin
			case(state1)
				0:
					begin
						ram_clk0 <= 0;
						ram_clk1 <= 0;
						iowr <= 0;
						iord <= 0;

						case(dop)
							NEXTLPMP:
								begin
									if(r[30][0] == 0)r[Rd] <= opcode0[7:0];
									else r[Rd] <= opcode0[15:8];
									{r[31],r[30]} <= {r[31],r[30]} + operand1;
									dop <= 0;
									PC <= operand3;
								end
							NEXTLPM:
								begin
									if(r[30][0] == 0)r[0] <= opcode0[7:0];
									else r[0] <= opcode0[15:8];
									dop <= 0;
									PC <= operand3;
								end
							NEXTRET:
								begin
									PC <= {RAM_DOUT1,RAM_DOUT0};
									dop <= 0;
								end
							NEXTRETI:
								begin
									PC <= {RAM_DOUT1,RAM_DOUT0};
									dop <= 0;
									sreg[SREG_I] <= 1;
								end
							NEXTLDS:
								begin
									r[Rd] <= RAM_DOUT0;
									dop <= 0;
									PC <= PC + 2;
								end
							NEXTLD:
								begin
									r[Rd] <= RAM_DOUT0;
									dop <= 0;
									PC <= PC +1;
								end
							NEXTIOW:
								begin
									r[`ADDRd] <= IODIN;
									dop <= 0;
									PC <= PC +1;
								end
							NEXTSBC:
								begin
									if(opcode0[9] == 1) iodout <= IODIN | (1 << opcode0[2:0]);
									else iodout <= IODIN & ~(1 << opcode0[2:0]);
									dop <= 0;
									obr <= out_c;
									PC <= PC + 1;
								end
							NEXTSBICS:
								begin
									if(IODIN[(opcode0[2:0])] == opcode0[9]) PC <= PC + 2 + comm_width;
									else PC <= PC + 1;
									dop <= 0;
								end
						default:
						begin
`ifdef irq_enable
						if(IRQ_REQ) //Perehod na vector
							begin
								ram_add0 <= sp - 96;
								ram_add1 <= sp - 97;
								{ram_din1,ram_din0} <= PC;				
								sp <= sp - 2;
								ram_we0 <= 1;		
								ram_we1 <= 1;
								sreg[SREG_I] <= 0;
                        PC <= IRQ_ADD;
                        obr <= rcall_c;
							end
						else
							begin
`endif
`ifdef debug
						//DEBUG
						OPCNT <= OPCNT + 1;
`endif
						case(o_num)
`ifdef c_adc							
							ADC:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= (r[`ADDRd][7] & r[`ADDRr][7])+(r[`ADDRr][7] & ~RALU[7])+(r[`ADDRd][7] & ~RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & r[`ADDRr][7] & ~RALU[7])+(~r[`ADDRd][7] & ~r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= (r[`ADDRd][3] & r[`ADDRr][3]) + (r[`ADDRr][3] & (~RALU[3])) + ((~RALU[3]) & r[`ADDRd][3]);
									sreg[SREG_S] <= RALU[7] ^ ((r[`ADDRd][7] & r[`ADDRr][7] & (~RALU[7]))+((~r[`ADDRd][7]) & (~r[`ADDRr][7]) & RALU[7]));
									PC <= PC + 1;
								end	
`endif
`ifdef c_add								
							ADD:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= (r[`ADDRd][7] & r[`ADDRr][7])+(r[`ADDRr][7] & ~RALU[7])+(r[`ADDRd][7] & ~RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & r[`ADDRr][7] & ~RALU[7])+(~r[`ADDRd][7] & ~r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= (r[`ADDRd][3] & r[`ADDRr][3]) + (r[`ADDRr][3] & (~RALU[3])) + ((~RALU[3]) & r[`ADDRd][3]);
									sreg[SREG_S] <= RALU[7] ^ ((r[`ADDRd][7] & r[`ADDRr][7] & (~RALU[7]))+((~r[`ADDRd][7]) & (~r[`ADDRr][7]) & RALU[7]));
									PC <= PC + 1;
								end
`endif 
`ifdef c_adiw							
							ADIW:
								begin
									r[{2'b11, opcode0[5:4], 1'b1}] <= RALU16[15:8];
									r[{2'b11, opcode0[5:4], 1'b0}] <= RALU16[7:0];
									sreg[SREG_V] <= (~r[`ADIWRd][7]) & RALU16[15];
									sreg[SREG_N] <= RALU16[15];
									sreg[SREG_Z] <= (RALU16 == 16'h0000)? 1 : 0;
									sreg[SREG_S] <= RALU16[15] ^ ((~r[`ADIWRd][7]) & RALU16[15]);
									sreg[SREG_C] <= (~RALU16[15]) & r[`ADIWRd][7];									
									PC <= PC + 1;
								end
`endif
`ifdef c_and									
							AND:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_V] <= 1'b0;
									sreg[SREG_S] <= 1'b0 ^ RALU[7];									
									PC <= PC + 1;
								end
`endif
`ifdef c_andi								
							ANDI:
								begin
									r[`CPIRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_V] <= 1'b0;
									sreg[SREG_S] <= 1'b0 ^ RALU[7];
									PC <= PC + 1;
								end
`endif
`ifdef c_asr
							ASR:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_V] <= r[`ADDRd][0] ^ RALU[7];
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= (r[`ADDRd][0] ^ RALU[7]) ^ RALU[7];
									sreg[SREG_C] <= r[`ADDRd][0];
									PC <= PC + 1;
								end
`endif
`ifdef c_bld
							BLD:
								begin	
									r[(opcode0[8:4])][(opcode0[2:0])] <= sreg[SREG_T];
									PC <= PC + 1;
								end
`endif							
`ifdef c_brbsc							
							BRBSC: //BRBC - BRBS
								begin	
									if(sreg[(opcode0[2:0])] == opcode0[10]) PC <= PC + 1;
									else //PC[6:0] <= PC[6:0] + opcode0[9:3] + 1;
										begin
											if(opcode0[9] == 1'b0) PC <= PC + opcode0[9:3] + 1;
											else PC <= PC - (128 - opcode0[9:3]) + 1;
										end
								end
`endif							
`ifdef c_bset
							BSET: //BCLR-BSET
								begin
									sreg[(opcode0[6:4])] <= ~opcode0[7];
									PC <= PC + 1;
								end
`endif							
`ifdef c_bst	

							BST:
								begin	
									sreg[SREG_T] <= r[(opcode0[8:4])][(opcode0[2:0])];
									PC <= PC + 1;
								end
`endif
`ifdef c_call									
							CALL:
								begin
									PC <= opcode1;
									ram_add0 <= sp - 96;
									ram_add1 <= sp - 97;
									{ram_din1,ram_din0} <= PC + 2;				
									sp <= sp - 2;
									ram_we0 <= 1;		
									ram_we1 <= 1;	
									obr <= rcall_c;
								end
`endif
`ifdef c_csbi
							CSBI: //CBI - SBI
								begin
									iocnt <= opcode0[7:3];
									dop <= NEXTSBC;
									obr <= in_c;
								end
`endif
`ifdef c_com									
							COM:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_V] <= 0;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= 0 ^ RALU[7];
									sreg[SREG_C] <= 1;
									PC <= PC + 1;
								end
`endif
`ifdef c_cp								
							CP:
								begin
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= ((~r[`ADDRd][7]) & r[`ADDRr][7])+(r[`ADDRr][7] & RALU[7])+((~r[`ADDRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`ADDRd][3]) & r[`ADDRr][3])+(r[`ADDRr][3] & RALU[3])+(RALU[3] & (~r[`ADDRd][3]));
									sreg[SREG_S] <= RALU[7]^((r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]));
									PC <= PC + 1;
								end	
`endif
`ifdef c_cpc									
							CPC:
								begin
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= ~RALU[0] & ~RALU[1] & ~RALU[2] & ~RALU[3] & ~RALU[4] & ~RALU[5] & ~RALU[6] & ~RALU[7] & sreg[SREG_Z];
									sreg[SREG_C] <= ((~r[`ADDRd][7]) & r[`ADDRr][7])+(r[`ADDRr][7] & RALU[7])+((~r[`ADDRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`ADDRd][3]) & r[`ADDRr][3])+(r[`ADDRr][3] & RALU[3])+((~r[`ADDRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]));
									PC <= PC + 1;
								end
`endif							
`ifdef c_cpi									
							CPI:
								begin
									sreg[SREG_H] <= ((~r[`CPIRd][3]) & operand[3])+(operand[3] & RALU[3])+((~r[`CPIRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]));
									sreg[SREG_V] <= (r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]);
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= ((~r[`CPIRd][7]) & operand[7])+(operand[7] & RALU[7])+((~r[`CPIRd][7]) & RALU[7]);
									PC <= PC + 1;
								end
`endif								
`ifdef c_cpse								
							CPSE:
								begin
									if(RALU == 0) PC <= PC + 2 + comm_width;
									else PC <= PC + 1;
								end
`endif								
`ifdef c_dec
							DEC:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_V] <= (~RALU[7]) & RALU[6] & RALU[5] & RALU[4] & RALU[3] & RALU[2] & RALU[1] & RALU[0];
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= RALU[7] ^ ((~RALU[7]) & RALU[6] & RALU[5] & RALU[4] & RALU[3] & RALU[2] & 
															RALU[1] & RALU[0]);
									PC <= PC + 1;
								end
`endif								
`ifdef c_eor							
							EOR:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_V] <= 1'b0;
									sreg[SREG_S] <= 1'b0 ^ RALU[7];									
									PC <= PC + 1;
								end
`endif
`ifdef c_fmul
							FMUL:
								begin
									r[1] <= RALU17[15:8];
									r[0] <= RALU17[7:0];
									sreg[SREG_C] <= RALU17[16];
									sreg[SREG_Z] <= (RALU17[15:0] == 16'h0000)? 1'b1 : 1'b0;
									PC <= PC + 1;
								end

`endif
`ifdef c_icall								
							ICALL:
								begin 
									PC[11:0] <= {r[31],r[30]};
									ram_add0 <= sp - 96;
									ram_add1 <= sp - 97;
									{ram_din1,ram_din0} <= PC + 1;				
									sp <= sp - 2;
									ram_we0 <= 1;		
									ram_we1 <= 1;	
									obr <= rcall_c;			
								end
`endif	
`ifdef c_ijmp								
							IJMP:
								begin 
									PC <= {r[31],r[30]};
								end
`endif
`ifdef c_in						
							IN:
								begin
									iocnt <= {opcode0[10:9],opcode0[3:0]};
									dop <= NEXTIOW;
									obr <= in_c;					
								end
`endif
`ifdef c_inc
							INC:
								begin
									r[(opcode0[8:4])] <= RALU;
									sreg[SREG_V] <= RALU[7] & (~RALU[6]) & (~RALU[5]) & (~RALU[4]) & (~RALU[3]) & (~RALU[2]) & 
														(~RALU[1]) & (~RALU[0]);
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= RALU[7] ^ (RALU[7]&(~RALU[6])&(~RALU[5])&(~RALU[4])&(~RALU[3])&(~RALU[2])&
															(~RALU[1])&(~RALU[0]));									
									PC <= PC + 1;
								end
`endif
`ifdef c_jmp								
							JMP:
								begin 
									PC <= {opcode1[15:0]};
								end
`endif
`ifdef c_ldi								
							LDI:
								begin 
									r[{1'b1,opcode0[7:4]}] <= {opcode0[11:8],opcode0[3:0]};
									PC <= PC + 1;
								end
`endif								
`ifdef c_lds									
							LDS:
								begin
									obr <= stx_c;
									Rd <= opcode0[8:4];
									ram_add0 <= opcode1 - 96;
									ram_we0 <= 0;
									dop <= NEXTLDS;
								end
`endif								
`ifdef c_ldx									
							LDX:
								begin
									Rd <= opcode0[8:4];
									obr <= stx_c;
									ram_add0 <= {r[27], r[26]}  - opcode0[1] - 96;
									{r[27], r[26]} <= {r[27], r[26]} + opcode0[0] - opcode0[1];
									ram_we0 <= 0;
									dop <= NEXTLD;
								end
`endif								
`ifdef c_ldyz								
							LDYZ:
								begin
									Rd <= opcode0[8:4];
									obr <= stx_c;
									if({opcode0[13],opcode0[11:10],opcode0[2:0]} == 0) ram_add0 <= {r[{3'b111,~opcode0[3],1'b1}], r[{3'b111,~opcode0[3],1'b0}]} - 96;
									else ram_add0 <= {r[{3'b111,~opcode0[3],1'b1}],r[{3'b111,~opcode0[3],1'b0}]} + {opcode0[13],opcode0[11:10],opcode0[2:0]} - 96;
									ram_we0 <= 0;
									dop <= NEXTLD;
								end
`endif
`ifdef c_ldypm								
							LDYPM:
								begin
									Rd <= opcode0[8:4];
									obr <= stx_c;
									ram_add0 <= {r[29], r[28]} - opcode0[1] - 96;
									{r[29], r[28]} <= {r[29], r[28]} + opcode0[0] - opcode0[1];
									ram_we0 <= 0;
									dop <= NEXTLD;
								end
`endif							
`ifdef c_ldzpm							
							LDZPM:
								begin
									Rd <= opcode0[8:4];
									obr <= stx_c;
									ram_add0 <= {r[31], r[30]} - opcode0[1] - 96;
									{r[31], r[30]} <= {r[31], r[30]} + opcode0[0] - opcode0[1];
									ram_we0 <= 0;
									dop <= NEXTLD;
								end
`endif								
`ifdef c_lpm									
							LPM:
								begin
									operand3 <= PC + 1;
									PC <= {r[31],r[30][7:1]};
									dop <= NEXTLPM;
								end
`endif						
`ifdef c_lpmz
							LPMZ:
								begin
									Rd <= opcode0[8:4];
									operand3 <= PC + 1;
									operand1[7:0] <= {7'b0000000,opcode0[0]};
									PC <= {r[31], r[30][7:1]};
									dop <= NEXTLPMP;
								end
`endif
`ifdef c_lsr								
							LSR:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_V] <= 0 ^ r[`ADDRd][0];
									sreg[SREG_N] <= 0;
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= (0^r[`ADDRd][0]) ^ 0;
									sreg[SREG_C] <= r[`ADDRd][0];								
									PC <= PC + 1;
								end
`endif								
`ifdef c_mov								
							MOV:
								begin
									r[(opcode0[8:4])] <= r[{opcode0[9], opcode0[3:0]}];
									PC <= PC + 1;
								end
`endif							
`ifdef c_movw								
							MOVW:
								begin
									r[{opcode0[7:4],1'b1}] <= r[{opcode0[3:0],1'b1}];
									r[{opcode0[7:4],1'b0}] <= r[{opcode0[3:0],1'b0}];
									PC <= PC + 1;
								end
`endif								
`ifdef c_mul									
							MUL:
								begin
									r[1] <= RALU16[15:8];
									r[0] <= RALU16[7:0];
									sreg[SREG_C] <= RALU16[15];
									sreg[SREG_Z] <= (RALU16 == 16'h0000)? 1'b1 : 1'b0;
									PC <= PC + 1;
								end
`endif							
`ifdef c_muls								
							MULS:
								begin
									r[1] <= RALU16[15:8];
									r[0] <= RALU16[7:0];
									sreg[SREG_C] <= RALU16[15];
									sreg[SREG_Z] <= (RALU16 == 16'h0000)? 1'b1 : 1'b0;
									PC <= PC + 1;
								end
`endif							
`ifdef c_mulsu								
							MULSU:
								begin
									r[1] <= RALU16[15:8];
									r[0] <= RALU16[7:0];
									sreg[SREG_C] <= RALU16[15];
									sreg[SREG_Z] <= (RALU16 == 16'h0000)? 1'b1 : 1'b0;
									PC <= PC + 1;
								end
`endif
`ifdef c_neg								
							NEG:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_H] <= RALU[3] + r[`ADDRd][3];
									sreg[SREG_V] <= RALU[7] & (~RALU[6]) & (~RALU[5]) & (~RALU[4]) & (~RALU[3]) & (~RALU[2]) & 
														(~RALU[1]) & (~RALU[0]);
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= RALU[7] ^ (RALU[7] & (~RALU[6]) & (~RALU[5]) & (~RALU[4]) & (~RALU[3]) & (~RALU[2]) & 
															(~RALU[1]) & (~RALU[0]));
									sreg[SREG_C] <= RALU[7] + RALU[6] + RALU[5] + RALU[4] + RALU[3] + RALU[2] + 
															RALU[1] + RALU[0];								
									PC <= PC + 1;
								end
`endif
`ifdef c_nop
							NOP:
								begin
									PC <= PC + 1;
								end
`endif
`ifdef c_or								
							OR:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_V] <= 1'b0;
									sreg[SREG_S] <= 1'b0 ^ RALU[7];	
									PC <= PC + 1;
								end
`endif							
`ifdef c_ori								
							ORI:
								begin 
									r[`CPIRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_V] <= 1'b0;
									sreg[SREG_S] <= 1'b0 ^ RALU[7];									
									PC <= PC + 1;
								end
`endif
`ifdef c_out								
							OUT:
								begin
									PC <= PC + 1;
									case({opcode0[10:9],opcode0[3:0]})
										SREGA: sreg <= r[(opcode0[8:4])];
										SPL: sp[7:0] <= r[(opcode0[8:4])];
										SPH: sp[15:8] <= r[(opcode0[8:4])];
										default: 
											begin
												iodout <= r[(opcode0[8:4])];
												iocnt <= {opcode0[10:9],opcode0[3:0]};
												obr <= out_c;
											end
									endcase
								end
`endif
`ifdef c_pop								
							POP:
								begin
									Rd <= opcode0[8:4];
									ram_add0 <= sp  - 96 + 1;
									sp <= sp + 1;
									ram_we0 <= 0;
									obr <= stx_c;
									dop <= NEXTLD;
								end
`endif								
`ifdef c_push									
							PUSH:
								begin
									ram_din0 <= r[(opcode0[8:4])];
									ram_add0 <= sp - 96;
									sp <= sp - 1;
									PC <= PC + 1;
									ram_we0 <= 1;
									obr <= stx_c;
								end
`endif								
`ifdef c_rcall								
							RCALL:
								begin 
									if(opcode0[11] == 0) PC <= PC + opcode0[11:0] + 1;
									else PC <= PC - (4096 - opcode0[11:0]) + 1;
									ram_add0 <= sp - 96;
									ram_add1 <= sp - 97;
									{ram_din1,ram_din0} <= PC + 1;				
									sp <= sp - 2;
									ram_we0 <= 1;		
									ram_we1 <= 1;	
									obr <= rcall_c;			
								end
`endif							
`ifdef c_ret									
							RET:
								begin
									ram_add1 <= sp - 96 + 1;
									ram_add0 <= sp - 96 + 2;
									sp <= sp + 2;	
									ram_we0 <= 0;
									ram_we1 <= 0;
									obr <= rcall_c;
									dop <= NEXTRET;
								end
`endif
`ifdef irq_enable
							RETI:
								begin
									ram_add1 <= sp - 96 + 1;
									ram_add0 <= sp - 96 + 2;
									sp <= sp + 2;	
									ram_we0 <= 0;
									ram_we1 <= 0;
									obr <= rcall_c;
									dop <= NEXTRETI;
								end
`endif
`ifdef c_rjmp								
							RJMP:
								begin
									if(opcode0[11] == 0) PC <= PC + opcode0[11:0] + 1;
									else PC <= PC - (4096 - opcode0[11:0]) + 1;
								end	
`endif
`ifdef c_ror									
							ROR:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_V] <= RALU[7]^r[`ADDRd][0];
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_S] <= RALU[7]^(RALU[7]^r[`ADDRd][0]);
									sreg[SREG_C] <= r[`ADDRd][0];									
									PC <= PC + 1;
								end
`endif
`ifdef c_sbc									
							SBC:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= ~RALU[0] & ~RALU[1] & ~RALU[2] & ~RALU[3] & ~RALU[4] & ~RALU[5] & ~RALU[6] & ~RALU[7] & sreg[SREG_Z];
									sreg[SREG_C] <= ((~r[`ADDRd][7]) & r[`ADDRr][7])+(r[`ADDRr][7] & RALU[7])+((~r[`ADDRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`ADDRd][3]) & r[`ADDRr][3])+(r[`ADDRr][3] & RALU[3])+((~r[`ADDRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]));
									PC <= PC + 1;
								end
`endif								
`ifdef c_sbci								
							SBCI:
								begin
									r[`CPIRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= ~RALU[0] & ~RALU[1] & ~RALU[2] & ~RALU[3] & ~RALU[4] & ~RALU[5] & ~RALU[6] & ~RALU[7] & sreg[SREG_Z];
									sreg[SREG_C] <= ((~r[`CPIRd][7]) & operand[7])+(operand[7] & RALU[7])+((~r[`CPIRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`CPIRd][3]) & operand[3])+(operand[3] & RALU[3])+((~r[`CPIRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]));
									PC <= PC + 1;								
								end
`endif							
`ifdef c_sbics									
							SBICS:
								begin
									iocnt <= opcode0[7:3];
									dop <= NEXTSBICS;
									obr <= in_c;
								end
`endif
`ifdef c_sbiw								
							SBIW:
								begin
									r[{2'b11, opcode0[5:4], 1'b1}] <= RALU16[15:8];
									r[{2'b11, opcode0[5:4], 1'b0}] <= RALU16[7:0];
									sreg[SREG_V] <= r[{2'b11, opcode0[5:4], 1'b1}][7] & (~RALU16[15]);
									sreg[SREG_N] <= RALU16[15];
									sreg[SREG_Z] <= (RALU16 == 16'h0000)? 1 : 0;
									sreg[SREG_S] <= RALU16[15] ^ (r[{2'b11, opcode0[5:4], 1'b1}][7] & (~RALU16[15]));
									sreg[SREG_C] <= RALU16[15] & (~r[{2'b11, opcode0[5:4], 1'b1}][7]);
									PC <= PC + 1;
								end
`endif
`ifdef c_sbrsc							
							SBRSC:
								begin	
									if(r[(opcode0[8:4])][(opcode0[2:0])] == opcode0[9]) PC <= PC + 2 + comm_width;
									else PC <= PC + 1;
								end
`endif														
`ifdef c_sts									
							STS:
								begin
									obr <= stx_c;
									ram_din0 <= r[(opcode0[8:4])];
									ram_add0 <= opcode1 - 96;
									PC <= PC + 2;
									ram_we0 <= 1;
								end
`endif								
`ifdef c_stx									
							STX:
								begin
									obr <= stx_c;
									ram_add0 <= {r[27], r[26]} - opcode0[1] - 96;
									{r[27], r[26]} <= {r[27], r[26]} - opcode0[1] + opcode0[0];
									ram_din0 <= r[(opcode0[8:4])];
									PC <= PC + 1;
									ram_we0 <= 1;
								end
`endif							
`ifdef c_styz									
							STYZ:
								begin
									obr <= stx_c;
									if({opcode0[13],opcode0[11:10],opcode0[2:0]} == 0) ram_add0 <= {r[{3'b111,~opcode0[3],1'b1}], r[{3'b111,~opcode0[3],1'b0}]} - 96;
									else ram_add0 <= {r[{3'b111,~opcode0[3],1'b1}],r[{3'b111,~opcode0[3],1'b0}]} + {opcode0[13],opcode0[11:10],opcode0[2:0]} - 96;
									ram_din0 <= r[(opcode0[8:4])];
									PC <= PC + 1;
									ram_we0 <= 1;
								end
`endif								
`ifdef c_stypm									
							STYPM:
								begin
									obr <= stx_c;
									ram_add0 <= {r[29], r[28]} - opcode0[1] - 96;
									{r[29], r[28]} <= {r[29], r[28]} - opcode0[1] + opcode0[0];
									ram_din0 <= r[(opcode0[8:4])];
									PC <= PC + 1;
									ram_we0 <= 1;
								end
`endif								
`ifdef c_stzpm									
							STZPM:
								begin
									obr <= stx_c;
									ram_add0 <= {r[31], r[30]} - opcode0[1] - 96;
									{r[31], r[30]} <= {r[31], r[30]} - opcode0[1] + opcode0[0];
									ram_din0 <= r[(opcode0[8:4])];
									PC <= PC + 1;
									ram_we0 <= 1;
								end
`endif							
`ifdef c_sub									
							SUB:
								begin
									r[`ADDRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= ((~r[`ADDRd][7]) & r[`ADDRr][7])+(r[`ADDRr][7] & RALU[7])+((~r[`ADDRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`ADDRd][3]) & r[`ADDRr][3])+(r[`ADDRr][3] & RALU[3])+((~r[`ADDRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`ADDRd][7] & (~r[`ADDRr][7]) & (~RALU[7]))+((~r[`ADDRd][7]) & r[`ADDRr][7] & RALU[7]));									
									PC <= PC + 1;
								end
`endif								
`ifdef c_subi								
							SUBI:
								begin
									r[`CPIRd] <= RALU;
									sreg[SREG_N] <= RALU[7];
									sreg[SREG_Z] <= (RALU == 0)? 1 : 0;
									sreg[SREG_C] <= ((~r[`CPIRd][7]) & operand[7])+(operand[7] & RALU[7])+((~r[`CPIRd][7]) & RALU[7]);
									sreg[SREG_V] <= (r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]);
									sreg[SREG_H] <= ((~r[`CPIRd][3]) & operand[3])+(operand[3] & RALU[3])+((~r[`CPIRd][3]) & RALU[3]);
									sreg[SREG_S] <= RALU[7]^((r[`CPIRd][7] & (~operand[7]) & (~RALU[7]))+((~r[`CPIRd][7]) & operand[7] & RALU[7]));
									PC <= PC + 1;
								end
`endif							
`ifdef c_swap									
							SWAP:
								begin
									r[`ADDRd] <= RALU;
									PC <= PC + 1;
								end
`endif
							default:
								begin
									;
								end
						endcase
`ifdef irq_enable
						end  //IRQ Enable
`endif
						end
						endcase
						
					end
				1:
					begin
						case(obr)
							in_c:
								begin
									iord <= 1;
									obr <= 0;				
								end
							out_c:
								begin
									iowr <= 1;	
									obr <= 0;				
								end							
																
							stx_c:
								begin
									ram_clk0 <= 1;	
									obr <= 0;				
								end
							rcall_c:
								begin
									ram_clk0 <= 1;
									ram_clk1 <= 1;	
									obr <= 0;				
								end

							default:
								begin
									;
								end
						endcase
					end		
			endcase
		end
end

//Decoder opcodov
always@(*)
	begin
		case(opcode0[15:12])
			4'h0:
				begin
					if(opcode0[11:0] == 12'd0) o_num = NOP;
					if(opcode0[11:10] == 2'b11) o_num = ADD;
					if(opcode0[11:10] == 2'b01) o_num = CPC;
					if(opcode0[11:10] == 2'b10) o_num = SBC;
					if(opcode0[11:8] == 4'b0001) o_num = MOVW;
					if(opcode0[11:8] == 4'b0010) o_num = MULS;
					if({opcode0[11:7],opcode0[3]} == 6'b001100) o_num = MULSU;
					if({opcode0[11:7],opcode0[3]} == 6'b001101) o_num = FMUL;
					if({opcode0[11:7],opcode0[3]} == 6'b001110) o_num = FMULS;
					if({opcode0[11:7],opcode0[3]} == 6'b001111) o_num = FMULSU;
				end
			4'h1:
				begin
					if(opcode0[11:10] == 2'b11) o_num = ADC;
					if(opcode0[11:10] == 2'b01) o_num = CP;
					if(opcode0[11:10] == 2'b10) o_num = SUB;
					if(opcode0[11:10] == 2'b00) o_num = CPSE;
				end
			4'h2:
				begin
					if(opcode0[11:10] == 2'b00) o_num = AND;
					if(opcode0[11:10] == 2'b10) o_num = OR;
					if(opcode0[11:10] == 2'b01) o_num = EOR;
					if(opcode0[11:10] == 2'b11) o_num = MOV;
				end
			4'h3: o_num = CPI;
			4'h4: o_num = SBCI;
			4'h5: o_num = SUBI;
			4'h6: o_num = ORI;
			4'h7: o_num = ANDI;
			4'h8:
				begin
					if(opcode0[9] == 1'b1) o_num = STYZ; //STDYPQ - STDZPQ - STYZ
					if(opcode0[9] == 1'b0) o_num = LDYZ; //LDDYPQ - LDDZPQ - LDYZ
				end
			4'h9:
				begin
					if(opcode0[11:10] == 2'b11) o_num = MUL;
					if({opcode0[11:9],opcode0[3:1]} == 6'b010110) o_num = JMP;
					if({opcode0[11:9],opcode0[3:1]} == 6'b010111) o_num = CALL;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100011) o_num = INC;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0101010) o_num = DEC;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100101) o_num = ASR;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100000) o_num = COM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100110) o_num = LSR;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100001) o_num = NEG;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100111) o_num = ROR;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0100010) o_num = SWAP;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011111) o_num = PUSH;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001111) o_num = POP;
					if(opcode0[11:0] == 12'h409) o_num = IJMP;
					if(opcode0[11:0] == 12'h508) o_num = RET;
					if(opcode0[11:0] == 12'h509) o_num = ICALL;
					if(opcode0[11:0] == 12'h5C8) o_num = LPM;
					if(opcode0[11:0] == 12'h518) o_num = RETI;
					if(opcode0[11:8] == 4'b0110) o_num = ADIW;
					if(opcode0[11:8] == 4'b0111) o_num = SBIW;
					if({opcode0[11:8],opcode0[3:0]} == 8'b01001000) o_num = BSET;  //BSET - BCLR
					if({opcode0[11:10],opcode0[8]} == 3'b100) o_num = CSBI; //CBI - SBI
					if({opcode0[11:10],opcode0[8]} == 3'b101) o_num = SBICS; 		//SBIC - SBIS
					if({opcode0[11:9],opcode0[3:0]} == 7'b0000000) o_num = LDS; 
					if({opcode0[11:9],opcode0[3:0]} == 7'b0010000) o_num = STS; 
					if({opcode0[11:9],opcode0[3:0]} == 7'b0000100) o_num = LPMZ; //LPMZ
					if({opcode0[11:9],opcode0[3:0]} == 7'b0000101) o_num = LPMZ; //LPMZ+ 
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001100) o_num = LDX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001101) o_num = LDX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001110) o_num = LDX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001001) o_num = LDYPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0001010) o_num = LDYPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0000001) o_num = LDZPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0000010) o_num = LDZPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011100) o_num = STX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011101) o_num = STX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011110) o_num = STX;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011001) o_num = STYPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0011010) o_num = STYPM;
					if({opcode0[11:9],opcode0[3:0]} == 7'b0010001) o_num = STZPM;	
					if({opcode0[11:9],opcode0[3:0]} == 7'b0010010) o_num = STZPM;	
				end
			4'hA:
				begin
					if(opcode0[9] == 1'b1) o_num = STYZ;
					if(opcode0[9] == 1'b0) o_num = LDYZ;
				end
			4'hB:
				begin
					if(opcode0[11]== 0) o_num = IN;
					if(opcode0[11]== 1) o_num = OUT;
				end
			4'hC: o_num = RJMP;
			4'hD: o_num = RCALL;
			4'hE: o_num = LDI;
			4'hF:
				begin
					if(opcode0[11]== 1'b0) o_num = BRBSC; //BRBC - BRBS
					if({opcode0[11:9],opcode0[3]}== 4'b1000) o_num = BLD;
					if({opcode0[11:9],opcode0[3]}== 4'b1010) o_num = BST;
					if({opcode0[11:10],opcode0[3]}== 3'b110) o_num = SBRSC; //SBRC - SBRS
					if(opcode0[11:0] == 12'hFFF) o_num = 0;
				end
		endcase
	end

	
wire ALU_CLK = hclk & state;
	
//ALU
always@(posedge ALU_CLK or negedge rst)	
begin
	if(!rst)
		begin
			RALU = 0;
			RALU16 = 0;
			operand = 0;
		end
	else
		begin
			case(o_num)
				ADC: 		RALU = r[(opcode0[8:4])] + r[{opcode0[9],opcode0[3:0]}] + sreg[SREG_C];
				ADD: 		RALU = r[(opcode0[8:4])] + r[{opcode0[9],opcode0[3:0]}];
				ADIW: 	RALU16 = {r[{2'b11, opcode0[5:4], 1'b1}],r[{2'b11, opcode0[5:4], 1'b0}]} + {opcode0[7:6],opcode0[3:0]};
				AND: 		RALU = r[(opcode0[8:4])] & r[{opcode0[9],opcode0[3:0]}];
				ANDI: 	RALU = r[{1'b1, opcode0[7:4]}] & {opcode0[11:8],opcode0[3:0]}; 
				ASR: 		RALU = {r[(opcode0[8:4])][7],r[(opcode0[8:4])][7:1]};
				CPI:  	begin RALU = r[{1'b1, opcode0[7:4]}] - {opcode0[11:8],opcode0[3:0]}; operand = {opcode0[11:8],opcode0[3:0]}; end
				COM: 		RALU = 8'hFF - r[(opcode0[8:4])];
				CPSE: 	RALU = r[(opcode0[8:4])] - r[{opcode0[9], opcode0[3:0]}];
				DEC: 		RALU = r[(opcode0[8:4])] - 1'b1;
				EOR: 		RALU = r[(opcode0[8:4])] ^ r[{opcode0[9],opcode0[3:0]}];
				INC: 		RALU = r[(opcode0[8:4])] + 1'b1;
				LSR: 		RALU =  {1'b0, r[(opcode0[8:4])][7:1]};
				MUL:  	RALU16 = $unsigned(r[(opcode0[8:4])]) * $unsigned(r[{opcode0[9], opcode0[3:0]}]);
				MULS: 	RALU16 = $signed(r[{1'b1,opcode0[7:4]}]) * $signed(r[{1'b1, opcode0[3:0]}]);
				MULSU: 	RALU16 = $signed(r[{2'b10,opcode0[6:4]}]) * $unsigned(r[{2'b10, opcode0[2:0]}]);				
				NEG: 		RALU =  8'h00 - r[(opcode0[8:4])];
				OR:  		RALU = r[(opcode0[8:4])] | r[{opcode0[9],opcode0[3:0]}];
				ORI:  	RALU = r[{1'b1, opcode0[7:4]}] | {opcode0[11:8],opcode0[3:0]};
				ROR: 		RALU = {sreg[SREG_C], r[(opcode0[8:4])][7:1]};			
				SBC: RALU = r[(opcode0[8:4])] - r[{opcode0[9],opcode0[3:0]}] - sreg[SREG_C];
				CPC: RALU = r[(opcode0[8:4])] - r[{opcode0[9],opcode0[3:0]}] - sreg[SREG_C];
				SBCI: 	begin RALU = r[{1'b1, opcode0[7:4]}] - {opcode0[11:8],opcode0[3:0]} - sreg[SREG_C]; operand = {opcode0[11:8],opcode0[3:0]}; end 
				SBIW: RALU16 = {r[{2'b11, opcode0[5:4], 1'b1}],r[{2'b11, opcode0[5:4], 1'b0}]} - {opcode0[7:6],opcode0[3:0]};
				SUB:  RALU = r[(opcode0[8:4])] - r[{opcode0[9],opcode0[3:0]}];
				CP:  RALU = r[(opcode0[8:4])] - r[{opcode0[9],opcode0[3:0]}];
				SUBI: begin RALU = r[{1'b1, opcode0[7:4]}] - {opcode0[11:8],opcode0[3:0]}; operand = {opcode0[11:8],opcode0[3:0]}; end 
				SWAP: RALU = {r[(opcode0[8:4])][3:0],r[(opcode0[8:4])][7:4]};
`ifdef c_fmul
				FMUL: 
					begin  
						RALU17[16] = r[{2'b10,opcode0[6:4]}][7] * r[{2'b10, opcode0[2:0]}][7];	
						RALU17[15:0] = r[{2'b10,opcode0[6:4]}][6:0] * r[{2'b10, opcode0[2:0]}][6:0];
					end
`endif
				default: begin RALU = 0; RALU16 = 0; operand = 0; `ifdef c_fmul RALU17 = 0; `endif end
			endcase
		end
end

endmodule
