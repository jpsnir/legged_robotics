% Computes gait timing variable s
% 
% Inputs: 
%       q1: cyclic variable
%       q1_min: minimum value for q1 during gait
%       q1_max: max value for gait
%
% Output: 
%       s: gait timing variable
%
function s = func_gait_timing(q1,q1_min,q1_max)

delq = q1_max - q1_min;

s = (q1 - q1_min)/delq;

end