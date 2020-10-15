`timescale 10ns/1ns

module mult_tp; //测试模块的名字
  reg[7:0] a,b; //测试输入信号定义为reg 型
  wire [15:0] out; //测试输出信号定义为wire 型
  integer i,j,k;
  mult8 m1(out,a,b); //调用测试对象
  //激励波形设定
  initial
  begin
    a=0;b=0;
    for(i=1;i<255;i=i+1)
      #10 a=i;
  end

  initial
  begin
    for(j=1;j<255;j=j+1)
      #10 b=j;
  end
  
    initial
  begin
    for(k=1;k<255;k=k+1)
      #10 b=k;
  end
  
  initial //定义结果显示格式
  begin
    $monitor($time,,,"%d * %d= %d",a,b,out);
      #2560 $finish;
  end
endmodule

module mult8(out, a, b); //8 位乘法器源代码
  parameter size=8;
  input[size:1] a,b; //两个操作数
  output[2*size:1] out; //结果
  assign out=a*b; //乘法运算符
endmodule