% Maps zero dynamics coordinates to full states using bezier polynomials
%
% Inputs:
%       z: [z, dz] = [q1, dq1]
%           stance leg angle and velocity
%       a: [alpha_2, alpha_3], bezier coefficients for q2 and q3
%           each has 5 coefficients for 4th order Bezier Poly
%       s_params: [q1_min, q1_max]
%           q1_min: minimum angle during gait
%           q1_max: maxmum angle during gait
%
% Outputs:
%       x: full dynamics [q, dq]
%

function x = func_map_z_x(z,a,s_params)

% Extracting info from inputs
q1 = z(1); dq1 = z(2);

alpha2 = a(1:5); alpha3 = a(6:end);

q1_min = s_params(1);
q1_max = s_params(2);

delq = q1_max - q1_min;

% Gait timing variable
s = func_gait_timing(q1,q1_min,q1_max);

% Bezier polynomial order
M = 4;

q2 = bezier(s,M,alpha2);
q3 = bezier(s,M,alpha3)';

dq2 = d_ds_bezier(s,M,alpha2)*dq1/delq;
dq3 = d_ds_bezier(s,M,alpha3)*dq1/delq;

% Arranging map
x = [z(1), q2, q3, z(2), dq2, dq3];

end