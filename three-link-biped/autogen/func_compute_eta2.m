function [eta2]= func_compute_eta2(q,dq,param)
%%%%%%  func_compute_eta2.m
%%%%  04/24/20
%%%%
%%%%
%%%%
%Inputs
q1=q(1);
q2=q(2);
q3=q(3);
%%%%
%%%%
dq1=dq(1);
dq2=dq(2);
dq3=dq(3);
%%%%
%%%%
r=param(1);
m=param(2);
Mh=param(3);
Mt=param(4);
l=param(5);
g=param(6);
%%%%
%%%%
%%%%
%%%%
eta2=zeros(1,1);
eta2(1,1) = (Mt*(2*dq1*l^2 + 2*dq3*l^2 + 2*dq1*r^2 - 4*dq1*l*r*cos(q3) - 2*dq3*l*r*cos(q3)))/2 + (dq1*m*r^2)/4 + (m*r^2*(10*dq1 + 2*dq2 - 8*dq1*cos(q2) - 4*dq2*cos(q2)))/8 + Mh*dq1*r^2;
%%%%
%%%%
%%End of code