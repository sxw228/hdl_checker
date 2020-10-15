module crc(crc_reg,crc,d,calc,init,d_valid,clk,reset);
  output[15:0] crc_reg;
  output[7:0] crc;
  input[7:0] d;
  input calc;
  input init;
  input d_valid;
  input clk;
  input reset;
  reg[15:0] crc_reg;
  reg[7:0] crc;
  wire[15:0] next_crc;
  integer i;
  
  always @(posedge clk or posedge reset)
  begin
    if (reset)
    begin
      crc_reg <= 16'h0000;
      crc <= 8'h00;
    end
    else if (init)
    begin
      crc_reg <= 16'h0000;
      crc <= 8'h00;
    end
    else if (calc & d_valid)
    begin
      crc_reg <= next_crc;
      crc <= ~{next_crc[8], next_crc[9], next_crc[10], next_crc[11],
      next_crc[12], next_crc[13], next_crc[14], next_crc[15]};
    end
    else if (~calc & d_valid)
    begin
      crc_reg <= {crc_reg[7:0], 8'h00};
      crc <= ~{crc_reg[0], crc_reg[1], crc_reg[2], crc_reg[3],
      crc_reg[4], crc_reg[5], crc_reg[6], crc_reg[7]};
    end
  end

  always @(posedge clk or posedge reset)
  begin
	for(i=0;i<2;i=i+1)
	  crc <= 8'h11;
  end

  assign next_crc[0] = crc_reg[12] ^ d[7] ^ crc_reg[8] ^ d[3];
  assign next_crc[1] = crc_reg[13] ^ d[6] ^ d[2] ^ crc_reg[9];
  assign next_crc[2] = d[5] ^ crc_reg[14] ^ d[1] ^ crc_reg[10];
  assign next_crc[3] = d[4] ^ crc_reg[15] ^ d[0] ^ crc_reg[11];
  assign next_crc[4] = crc_reg[12] ^ d[3];
  assign next_crc[5]=crc_reg[12]^crc_reg[13]^d[7]^crc_reg[8]^d[2]^d[3];
  assign next_crc[6] = crc_reg[13] ^ d[6] ^ crc_reg[14] ^ d[1] ^ d[2] ^
  crc_reg[9];
  assign next_crc[7] = d[5] ^ crc_reg[14] ^ crc_reg[15] ^ d[0] ^ d[1] ^
  crc_reg[10];
  assign next_crc[8] = d[4] ^ crc_reg[15] ^ d[0] ^ crc_reg[0] ^ crc_reg[11];
  assign next_crc[9] = crc_reg[12] ^ crc_reg[1] ^ d[3];
  assign next_crc[10] = crc_reg[13] ^ d[2] ^ crc_reg[2];
  assign next_crc[11] = crc_reg[3] ^ crc_reg[14] ^ d[1];
  assign next_crc[12] = crc_reg[12] ^ crc_reg[4] ^ d[7] ^ crc_reg[15] 
  ^ d[0] ^ crc_reg[8] ^ d[3];
  assign next_crc[13] = crc_reg[13] ^ d[6] ^ crc_reg[5] ^ d[2] ^ crc_reg[9];
  assign next_crc[14] = d[5] ^ crc_reg[14] ^ crc_reg[6] ^ d[1] ^ crc_reg[10];
  assign next_crc[15] = d[4] ^ crc_reg[15] ^ d[0] ^ crc_reg[7] ^ crc_reg[11];
endmodule