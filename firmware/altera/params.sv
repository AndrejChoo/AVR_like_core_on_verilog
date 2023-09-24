//Defines
`define ADDRr {opcode0[9], opcode0[3:0]}
`define ADDRd (opcode0[8:4])
`define CPIRd {1'b1, opcode0[7:4]}
`define ADIWRr {2'b11, opcode0[5:4], 1'b0}
`define ADIWRd {2'b11, opcode0[5:4], 1'b1}

//Parameters
parameter FSIZE = 4096;
parameter RSIZE = 1024;
//Settings
localparam ROM_ADD_WIDTH = $clog2(FSIZE) - 1;
localparam RAM_ADD_WIDTH = $clog2(RSIZE) - 1;

//Opcodes
parameter ADD	= 8'h1; //+
parameter ADC	= 8'h2; //+
parameter ADIW	= 8'h3; //+
parameter AND	= 8'h4; //+
parameter ANDI	= 8'h5; //+
parameter ASR	= 8'h6; //+
parameter BLD	= 8'h7; //+
parameter BRBSC	= 8'h8; //+
parameter BSET	= 8'h9; //+
parameter BST	= 8'hA; //+
parameter CALL	= 8'hB; //+
parameter CSBI	= 8'hC; //+
parameter COM	= 8'hD; //+
parameter CP	= 8'hE; //+
parameter CPC	= 8'hF; //+
parameter CPI	= 8'h10; //+
parameter CPSE	= 8'h11; //+
parameter DEC	= 8'h12; //+
parameter EOR	= 8'h13; //+
parameter IN	= 8'h14; //+
parameter INC	= 8'h15; //+
parameter IJMP = 8'h16; //+
parameter ICALL = 8'h17; //+
parameter JMP	= 8'h18; //+
parameter LDDPQ = 8'h19; //+
parameter LDI	= 8'h1A; //+
parameter LDS	= 8'h1B; //+
parameter LDX = 8'h1C; //+
parameter LDYZ = 8'h1D; //+
parameter LDYPM = 8'h1E; //+
parameter LDZPM = 8'h1F; //+
parameter LPM	= 8'h20; //+
parameter LPMZ = 8'h21; //+
parameter LSR	= 8'h22; //+
parameter MOV	= 8'h23;//+
parameter MOVW = 8'h24; //+
parameter MUL	= 8'h25; //+
parameter MULS = 8'h26; //+
parameter MULSU = 8'h27; //+
parameter NOP	= 8'h28; //+
parameter NEG	= 8'h29; //+
parameter OR	= 8'h2A; //+
parameter ORI	= 8'h2B; //+
parameter OUT	= 8'h2C; //+
parameter POP = 8'h2D; //+
parameter PUSH = 8'h2E; //+
parameter RCALL = 8'h2F; //+
parameter RET	= 8'h30; //+
parameter RETI	= 8'h31;
parameter RJMP	= 8'h32; //+
parameter ROR	= 8'h33; //+
parameter SBC	= 8'h34; //+
parameter SBCI	= 8'h35; //+
parameter SBICS	= 8'h36; //+
parameter SBIW	= 8'h37; //+
parameter SBRSC	= 8'h38; //+
parameter STDPQ = 8'h39; //+
parameter STS = 8'h3A; //+
parameter STX = 8'h3B; //+
parameter STYZ = 8'h3C; //+
parameter STYPM = 8'h3D; //+
parameter STZPM = 8'h3E; //+
parameter SUB	= 8'h3F; //+
parameter SUBI	= 8'h40; //+
parameter SWAP	= 8'h41; //+

//Obrabotchik
parameter in_c = 1;
parameter out_c = 2;
parameter stx_c = 3;
parameter rcall_c = 4;

//Dop 
parameter NEXTIOW = 1; //+
parameter NEXTSBICS = 2; //+
parameter NEXTLDS = 3; //+
parameter NEXTLPM = 4; //+
parameter NEXTLD  = 5; //+
parameter NEXTRETI = 6; //+
parameter NEXTLPMP = 7; //+
parameter NEXTRET = 8; //+
parameter NEXTSBC = 9; //+

//Sreg Flags
parameter SREG_C = 0;	
parameter SREG_Z = 1;
parameter SREG_N = 2;
parameter SREG_V = 3;
parameter SREG_S = 4;
parameter SREG_H = 5;
parameter SREG_T = 6;
parameter SREG_I = 7;

//IO REGS
localparam	SREGA = 8'h3F;
localparam	SPL	= 8'h3D;
localparam	SPH	= 8'h3E;

//Compill COMMANDS

`define c_add
`define c_adc
`define c_adiw
`define c_and
`define c_andi
`define c_asr
`define c_bld
`define c_brbsc
`define c_bset
`define c_bst
//`define c_call
`define c_csbi
`define c_com
`define c_cp
`define c_cpc
`define c_cpi
`define c_cpse
`define c_dec
`define c_eor
//`define c_fmul
`define c_icall
`define c_ijmp
`define c_in
`define c_inc
//`define c_jmp
`define c_ldi
`define c_lddpq
`define c_lds
`define c_ldx
`define c_ldyz
`define c_ldypm
`define c_ldzpm
`define c_lpm
`define c_lpmz
`define c_lsr
`define c_mov
`define c_movw
`define c_mul
//`define c_muls
//`define c_mulsu
`define c_neg
`define c_nop
`define c_or
`define c_ori
`define c_out
`define c_pop
`define c_push
`define c_rcall
`define c_ret
`define c_rjmp
`define c_ror
`define c_sbc
`define c_sbci
`define c_sbics
`define c_sbiw
`define c_sbrsc
`define c_stdpq
`define c_sts
`define c_stx
`define c_styz
`define c_stypm
`define c_stzpm
`define c_sub
`define c_subi
`define c_swap

//Interrupts ENABLE-DISABLE
`define irq_enable
