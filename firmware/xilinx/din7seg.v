module din7seg 
#(
 parameter razr_val = 8, //Количество разрядов  от 2 до 9 
 parameter in_clock = 50_000_000, //Входная частота
 parameter din_clock = razr_val*50, //Частота динамической индикации
 parameter clk_val = in_clock/din_clock/2-1, //Делитель частоты
 parameter reg_val = $clog2(clk_val)  //Разрядность делителя 
)
(
input wire clk,
input wire[3:0]I0,
input wire[3:0]I1,
input wire[3:0]I2,
input wire[3:0]I3,
input wire[3:0]I4,
input wire[3:0]I5,
input wire[3:0]I6,
input wire[3:0]I7,
input wire[3:0]I8,
output reg[7:0]SEG,
output reg[(razr_val - 1):0]RAZR
);


reg[3:0]O;
reg[3:0]C;
reg[(reg_val-1):0]DIV_CNT;
reg clock;
 
 
 always@(posedge clk)
	begin
		DIV_CNT <= DIV_CNT+1;
		if(DIV_CNT == clk_val)
			begin
				DIV_CNT <= 0;
				clock<=~clock;
			end
	end
 
 always@(posedge clock)
   begin 
	  C <= C+1'b1;
	  if(C==(razr_val-1)) C<=0;
	end
 
 always@(C)
  begin
   case(C)
	 4'b0000: begin O <= I0; RAZR <= 9'b111111110; end
	 4'b0001: begin O <= I1; RAZR <= 9'b111111101; end
	 4'b0010: begin O <= I2; RAZR <= 9'b111111011; end
	 4'b0011: begin O <= I3; RAZR <= 9'b111110111; end
	 4'b0100: begin O <= I4; RAZR <= 9'b111101111; end
	 4'b0101: begin O <= I5; RAZR <= 9'b111011111; end
	 4'b0110: begin O <= I6; RAZR <= 9'b110111111; end
	 4'b0111: begin O <= I7; RAZR <= 9'b101111111; end
	 4'b1000: begin O <= I8; RAZR <= 9'b011111111; end
	 default: begin O <= 0;  RAZR <= 9'b111111111; end 
	endcase
 end
 
 //DECODER
  always@(O)
  begin
   case(O)
     4'b0000: SEG <= 8'b00111111;//0
     4'b0001: SEG <= 8'b00000110;//1
     4'b0010: SEG <= 8'b01011011;//2
     4'b0011: SEG <= 8'b01001111;//3
     4'b0100: SEG <= 8'b01100110;//4
     4'b0101: SEG <= 8'b01101101;//5
     4'b0110: SEG <= 8'b01111101;//6
     4'b0111: SEG <= 8'b00000111;//7
     4'b1000: SEG <= 8'b01111111;//8
     4'b1001: SEG <= 8'b01101111;//9
     4'b1010: SEG <= 8'b01110111;//A
     4'b1011: SEG <= 8'b01111100;//B
     4'b1100: SEG <= 8'b00111001;//C
     4'b1101: SEG <= 8'b01011110;//D
     4'b1110: SEG <= 8'b01111001;//E
     4'b1111: SEG <= 8'b01110001;//F
	  
   endcase
  end
 
endmodule






