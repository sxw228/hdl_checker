module MAC(out,opa,opb,clk,clr);
  output[15:0] out;
  input[7:0] opa,opb;
  input clk,clr;
  wire[15:0] sum;
  reg[15:0] out;
  
  function[15:0] mult; //函数定义，mult 函数完成乘法操作
    input[7:0] opa,opb; //函数只能定义输入端，输出端口为函数名本身
    reg[15:0] result;
    integer i;
  begin
    result = opa[0]? opb : 0;
    for(i= 1; i <= 7; i = i+1)
    begin
      if(opa[i]==1) result=result+(opb<<(i-1));
    end
    mult=result;
  end
  endfunction
  
  assign sum=mult(opa,opb)+out;
  
  always @(posedge clk or posedge clr)
  begin
    if(clr) out<=0;
    else out<=sum;
  end
endmodule

module mac_tp;
  reg[7:0] opa,opb; //测试输入信号用reg 型变量
  reg clr,clk;
  wire[15:0] out; //测试输出信号用wire 型变量
  parameter DELY = 100;
  integer i;
  
  //测试对象调用
  MAC m1(out,opa,opb,clk,clr);

  always #(DELY) clk = ~clk; //产生时钟波形
  
  initial begin
    for(i=0;i<10;i=i+1)
	  #DELY opa=8'd12;
  end
  
  initial begin //激励波形定义
    clr=1;clk=0;opa=8'd0; opb=8'd0;
    #DELY clr=0;opa=8'd1; opb=8'd10;
    #DELY opa=8'd2; opb=8'd10;
    #DELY opa=8'd3; opb=8'd10;
    #DELY opa=8'd4; opb=8'd10;
    #DELY opa=8'd5; opb=8'd10;
    #DELY opa=8'd6; opb=8'd10;
    #DELY opa=8'd7; opb=8'd10;
    #DELY opa=8'd8; opb=8'd10;
    #DELY opa=8'd9; opb=8'd10;
    #DELY opa=8'd10; opb=8'd10;
    #DELY $finish;
  end
  
  //结果显示
  initial $monitor($time,,,"clr=%b opa=%d opb=%d out=%d",clr,opa,opb,out);
endmodule