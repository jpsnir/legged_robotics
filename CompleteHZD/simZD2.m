deg2rad = pi/180;
rad2deg = 1/deg2rad;

%   f = [q10, dq10, alpha(3-5)_q2, alpha(3-5)_q3]
%       q10: pre-impact inital angle for q1
%       dq10: pre-impact inital velocity for dq1
%       alpha(3-5)_q2:
%                   3rd to 5th Bezier coefficient for q2
%       alpha(3-5)_q3:
%                   3rd to 5th Bezier coefficient for q3

z_tot = [];

n = 5;

for i = 1:n
    
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
    
    f(1:2) = z_sol(end,:);
    
    z_tot = [z_tot; z_sol]; %#ok
    
end

figure
plot(z_tot(1,1)*rad2deg,z_tot(1,2)*rad2deg,'x'), hold on
plot(z_tot(:,1)*rad2deg,z_tot(:,2)*rad2deg)
hold off, grid on
title('Phase portrait of q_1 vd dq_1')
xlabel('q_1 (deg)')
ylabel('dq_1 (deg/s)')
legend('Start','Trajectory','Location','Best')

if bez==1
    alpha2 = [-f(5)+2*f(3), -f(4)+2*f(3), f(3:5)]
    alpha3 = [-f(8)+2*f(6), -f(7)+2*f(6), f(6:8)]

    M = 4;
    s = linspace(0,1,20);
    ss = linspace(0,1,length(alpha2));

    b2 = bezier(s,M,alpha2);
    b3 = bezier(s,M,alpha3);

    figure(),hold on
    title('Bezier Curve for q_2')
    plot(s,b2*rad2deg,'b-')
    plot(ss,alpha2*rad2deg,'r--o','MarkerFaceColor','r')
    xlabel('Gait Timing Variable, s');
    ylabel('\alpha_2 Coefficients (deg)');
    legend('Bezier Curve', '\alpha_2 Coefficients','Location','Best')

    figure(),hold on
    title('Bezier Curve for q_3')
    plot(s,b3*rad2deg,'b-')
    plot(ss,alpha3*rad2deg,'r--o','MarkerFaceColor','r')
    xlabel('Gait Timing Variable, s');
    ylabel('\alpha_3 Coefficients (deg)');
    legend('Bezier Curve', '\alpha_3 Coefficients','Location','Best')
end
