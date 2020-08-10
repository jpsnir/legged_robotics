function [De,E,dY_dq]= func_compute_De_E_dY_dq(q,dq,param)
%%%%%%  func_compute_De_E_dY_dq.m
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
De=zeros(5,5);
De(1,1) = Mt*l^2 + Mh*r^2 + Mt*r^2 + (3*m*r^2)/2 - m*r^2*cos(q2) - 2*Mt*l*r*cos(q3);
De(1,2) = -(m*r^2*(2*cos(q2) - 1))/4;
De(1,3) = Mt*l*(l - r*cos(q3));
De(1,4) = Mt*l*cos(q1 + q3) - (3*m*r*cos(q1))/2 + (m*r*cos(q1 + q2))/2 - Mh*r*cos(q1) - Mt*r*cos(q1);
De(1,5) = Mt*l*sin(q1 + q3) - Mt*r*sin(q1) - (3*m*r*sin(q1))/2 - Mh*r*sin(q1) + (m*r*sin(q1 + q2))/2;
De(2,1) = -(m*r^2*(2*cos(q2) - 1))/4;
De(2,2) = (m*r^2)/4;
De(2,3) = 0;
De(2,4) = (m*r*cos(q1 + q2))/2;
De(2,5) = (m*r*sin(q1 + q2))/2;
De(3,1) = Mt*l*(l - r*cos(q3));
De(3,2) = 0;
De(3,3) = Mt*l^2;
De(3,4) = Mt*l*cos(q1 + q3);
De(3,5) = Mt*l*sin(q1 + q3);
De(4,1) = Mt*l*cos(q1 + q3) - (3*m*r*cos(q1))/2 + (m*r*cos(q1 + q2))/2 - Mh*r*cos(q1) - Mt*r*cos(q1);
De(4,2) = (m*r*cos(q1 + q2))/2;
De(4,3) = Mt*l*cos(q1 + q3);
De(4,4) = Mh + Mt + 2*m;
De(4,5) = 0;
De(5,1) = Mt*l*sin(q1 + q3) - Mt*r*sin(q1) - (3*m*r*sin(q1))/2 - Mh*r*sin(q1) + (m*r*sin(q1 + q2))/2;
De(5,2) = (m*r*sin(q1 + q2))/2;
De(5,3) = Mt*l*sin(q1 + q3);
De(5,4) = 0;
De(5,5) = Mh + Mt + 2*m;
%%%%
%%%%
E=zeros(2,5);
E(1,1) = r*(cos(q1 + q2) - cos(q1));
E(1,2) = r*cos(q1 + q2);
E(1,3) = 0;
E(1,4) = 1;
E(1,5) = 0;
E(2,1) = r*(sin(q1 + q2) - sin(q1));
E(2,2) = r*sin(q1 + q2);
E(2,3) = 0;
E(2,4) = 0;
E(2,5) = 1;
%%%%
%%%%
dY_dq=zeros(2,5);
dY_dq(1,1) = r*(cos(q1 + q2) - cos(q1));
dY_dq(1,2) = r*cos(q1 + q2);
dY_dq(1,3) = 0;
dY_dq(1,4) = 1;
dY_dq(1,5) = 0;
dY_dq(2,1) = r*(sin(q1 + q2) - sin(q1));
dY_dq(2,2) = r*sin(q1 + q2);
dY_dq(2,3) = 0;
dY_dq(2,4) = 0;
dY_dq(2,5) = 1;
%%%%
%%%%
%%End of code