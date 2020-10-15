`timescale 10ns/1ns
module random_tp;
  integer data;
  integer i,j;
  parameter delay=10;
  
  initial $monitor($time,,,"data=%b",data);
  
  initial begin
    for(i=0; i<=100; i=i+1)
      #delay data=$random; //每次产生一个随机数
  end
  
  initial begin
    for(j=0; j<=100; j=j+1)
      #delay data=$random; //每次产生一个随机数
  end
  
endmodule