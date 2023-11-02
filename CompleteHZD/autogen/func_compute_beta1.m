function [beta1]= func_compute_beta1(q,dq,param)
%%%%%%  func_compute_beta1.m
%%%%  04/24/20
%%%%
%%%%
%%%%
%Inputs
s=q(1);
%%%%
%%%%
dq1=dq(1);
delq=dq(2);
%%%%
%%%%
a21=param(1);
a22=param(2);
a23=param(3);
a24=param(4);
a25=param(5);
a31=param(6);
a32=param(7);
a33=param(8);
a34=param(9);
a35=param(10);
%%%%
%%%%
%%%%
%%%%
beta1=zeros(2,1);
beta1(1,1) = -(dq1^2*(3*s^2*(4*a24 - 4*a25) - s^2*(12*a23 - 12*a24) - 3*(s - 1)^2*(4*a21 - 4*a22) + (s - 1)^2*(12*a22 - 12*a23) - 2*s*(s - 1)*(12*a23 - 12*a24) + s*(2*s - 2)*(12*a22 - 12*a23)))/delq^2;
beta1(2,1) = -(dq1^2*(3*s^2*(4*a34 - 4*a35) - s^2*(12*a33 - 12*a34) - 3*(s - 1)^2*(4*a31 - 4*a32) + (s - 1)^2*(12*a32 - 12*a33) - 2*s*(s - 1)*(12*a33 - 12*a34) + s*(2*s - 2)*(12*a32 - 12*a33)))/delq^2;
%%%%
%%%%
%%End of code