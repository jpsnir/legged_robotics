% Optimize intial conditions for ZD states and Bezier coeffs using fmincon
%
% Zero dynamics simulator takes pre-impact condition so that should be I.C.
% for fmincon as well

%------------------------------------------------------------------------%

beta = 180 - 2*theta;
q1_minus = theta-90;
q2f = beta;
q2df = 180 - abs(q1d);

% Bezier coefficients
n = 5; % Weighting factor for weighted average

a5 = q2f; 
a4 = a5 - (1/4)*q2df;
% a3 = 0;%(max([a4,a5])+n*min([a4,a5]))/(n+1);%180

g5 = q3f;
g4 = g5 - (1/4)*q3df; 
% g3 = 150;%(max([g4,g5])+n*min([g4,g5]))/(n+1);


z_minus = [q1_minus, q1d]*pi/180;
alpha =  [ a3, a4, a5]*pi/180;    %for q2 - alpha 3rd - 5th [~, rad, rad/s]
gamma =  [ g3, g4, g5]*pi/180;    %for q3 - alpha 3rd - 5th [~, rad, rad/s]

f0 = [z_minus, alpha, gamma];   %parameters that need to be optimized

% f0 = [-0.3478   -3.0765    0.1520    0.3604    0.5711    2.8535    2.8806    3.2398];

%------------------------------------------------------------------------%

% Lower and upper bounds for optimizer - [q1 q1_dot, alpha q2, alpha q3]
%
% epsilon for angles
eps = 5*pi/180;
% epsilon for velocity
eps_v = 1;
lb = [z_minus(1)-eps, z_minus(2)-eps_v, alpha-eps, gamma-eps];
ub = [z_minus(1)+eps, z_minus(2)+eps_v, alpha+eps, gamma+eps];

%------------------------------------------------------------------------%

%%%% Run optimizer

% Options:
opts = optimoptions('fmincon','Algorithm','interior-point',...
                    'MaxIterations',150,...
                    'StepTolerance',1e-6,...
                    'FunctionTolerance',1e-2);
opts.Display = 'iter';


% using only bounds and nonlinear contraints
%
% Outputs: f - optimized parameters [qt, dqt, qt_LR_max, alphaR, alphaL]
%          J - Cost to minimize
%
[f, J] = fmincon(@(f) pass_cost_ZD(f), f0, [],[],[],[], lb,ub, @confuneq,opts);

disp('solution found!');

f

% Function only exists to pass cost as a scalar output for fmincon, actual
% cost calculation happends in func_cost_ZD
%
function J = pass_cost_ZD(y)

[J, ~] = func_cost_ZD(y);

end

%------------------------------------------------------------------------%

% Simulate a single step of zero dynamics and get cost
%
% Inputs: [z_plus, a, s_params]
%       z_plus: [q1, dq1] post impact condition
%       a: [alpha2, alpha3] Bezier coefficients for q2 and q3
%       s_params: [q1_min, q1_max]
%
% Outputs: [J, z_sol]
%       J: Cost
%       z_sol: solution to zero dynamics ODE45
%
function [J, z_sol] = func_cost_ZD(f)

%%%% Simulate

% Simulation of a single step of ZD to using preimpact conditions
%   applies impact first then simulates zero dynamics
%
% Inputs: f = [q10, dq10, alpha(3-5)_q2, alpha(3-5)_q3]
%       q10: pre-impact inital angle for q1
%       dq10: pre-impact inital velocity for dq1
%       alpha(3-5)_q2:
%                   3rd to 5th Bezier coefficient for q2
%       alpha(3-5)_q3:
%                   3rd to 5th Bezier coefficient for q3
%
% Outputs:
%       t_sol - time (s) of zero dynamics
%       z_sol - [q1, dq1] post impact dynamics
%

[t_sol, z_sol] = sim_zero_dynamics(f);

%%%% Compute cost

% Variables needed to compute control action
q1_min = -f(1);
q1_max = f(1);

s_params = [q1_min, q1_max];

alpha2 = [-f(5)+2*f(3), -f(4)+2*f(3), f(3:5)];
alpha3 = [-f(8)+2*f(6), -f(7)+2*f(6), f(6:8)];

a = [alpha2, alpha3];

J = 0;
for i = 1:length(t_sol)
    % Post compute control action
    %
    % Inputs: [z, a, s_params]
    %       z: [q1, dq1]
    %       a: [alpha2, alpha3] - 1st to 5th Bezier coefficient for q2 and q3
    %       s_params: [q1_min, q1_max]
    %
    % Output:
    %       u: control action
    %
    
    u = func_compute_control_action(z_sol(i,:),a,s_params);
    
    % Cost = sum of norms of control action
    J = J + norm(u).^2;
end

end

%------------------------------------------------------------------------%

% Nonlinear constraint for optimizer
%
% Inputs: f: [q1, dq1, alpha2(3:5), alpha3(3:5)]
%
% Output: [c, ceq]
%       c: nonlinear inequality contraint
%       ceq: nonlinear equality contraint
%
function [c,ceq] = confuneq(f)

z_i = f(1:2);       %Pre-impact states

% Simulation of a single step of ZD to using preimpact conditions
%   applies impact first then simulates zero dynamics
%
% Inputs: f = [q10, dq10, alpha(3-5)_q2, alpha(3-5)_q3]
%       q10: pre-impact inital angle for q1
%       dq10: pre-impact inital velocity for dq1
%       alpha(3-5)_q2:
%                   3rd to 5th Bezier coefficient for q2
%       alpha(3-5)_q3:
%                   3rd to 5th Bezier coefficient for q3
%
% Outputs:
%       t_sol - time (s) of zero dynamics
%       z_sol - [q1, dq1] post impact dynamics
%
[~, z_sol] = sim_zero_dynamics(f);

z_f = z_sol(end,:);     %final values after swing phase and right before the next impact

% Nonlinear inequality constraints
c = [];
% Nonlinear equality constraints
ceq = [norm(z_i(1) - z_f(1)); norm(z_i(2) - z_f(2));]; 
% Force pre-impact condition and final solution of ODE45 to be the same

end

