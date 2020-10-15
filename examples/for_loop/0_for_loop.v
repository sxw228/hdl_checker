module decoder2(c,y,clk);
  output[6:0] c; //c 为输出码字，c[6]为高次项
  input[6:0] y; //y 为接收码字，y[6]为高次项
  input clk;
  reg[6:0] c,c_buf,buffer;
  reg temp;
  reg s0,s1,s2; //伴随式电路寄存器
  reg e; //错误检测输出信号
  integer i,j;
  
  always @(posedge clk)
  begin
    for (j=0;j<3;j=j+1)
	  c_buf[6]=0;
  end
  
  always @(posedge clk)
  begin
    s0=0; s1=0; s2=0; //初始化
    temp=0;
    buffer=y; //接收码字移入缓存
    for (i=6;i>=0;i=i-1) //接收码字进入除法电路
    begin
      e=s0&(~s1)&temp;
      temp=s2;
      s2=s1;
      s1=s0^temp;
      s0=y[i]^temp^e;
    end
    for (i=6;i>=0;i=i-1) //输出纠错译码后的码字
    begin
      e=s0&(~s1)&temp;
      temp=s2;
      s2=s1;
      s1=s0^temp;
      s0=temp^e;
      c_buf[i]=buffer[i]^e;
      if (e==1) //若出错，对缓存进行清零
      begin
        s0=0; s1=0; s2=0;
      end
    end
  end

  always @(posedge clk)
  begin
      c=c_buf;
  end
endmodule