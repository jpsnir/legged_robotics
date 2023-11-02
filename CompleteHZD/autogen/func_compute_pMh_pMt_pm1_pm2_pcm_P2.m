function [pMh,pMt,pm1,pm2,pcm,p2]= func_compute_pMh_pMt_pm1_pm2_pcm_p2(q,dq,param)
%%%%%%  func_compute_pMh_pMt_pm1_pm2_pcm_p2.m
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
pMh=zeros(2,1);
pMh(1,1) = -r*sin(q1);
pMh(2,1) = r*cos(q1);
%%%%
%%%%
pMt=zeros(2,1);
pMt(1,1) = l*sin(q1 + q3) - r*sin(q1);
pMt(2,1) = r*cos(q1) - l*cos(q1 + q3);
%%%%
%%%%
pm1=zeros(2,1);
pm1(1,1) = -(r*sin(q1))/2;
pm1(2,1) = (r*cos(q1))/2;
%%%%
%%%%
pm2=zeros(2,1);
pm2(1,1) = (r*(sin(q1 + q2) - 2*sin(q1)))/2;
pm2(2,1) = -(r*(cos(q1 + q2) - 2*cos(q1)))/2;
%%%%
%%%%
pcm=zeros(2,1);
pcm(1,1) = (Mt*(l*sin(q1 + q3) - r*sin(q1)) - Mh*r*sin(q1) - (m*r*sin(q1))/2 + (m*r*(sin(q1 + q2) - 2*sin(q1)))/2)/(Mh + Mt + 2*m);
pcm(2,1) = -(Mt*(l*cos(q1 + q3) - r*cos(q1)) - (m*r*cos(q1))/2 + (m*r*(cos(q1 + q2) - 2*cos(q1)))/2 - Mh*r*cos(q1))/(Mh + Mt + 2*m);
%%%%
%%%%
p2=zeros(2,1);
p2(1,1) = r*(sin(q1 + q2) - sin(q1));
p2(2,1) = -r*(cos(q1 + q2) - cos(q1));
%%%%
%%%%
%%End of code