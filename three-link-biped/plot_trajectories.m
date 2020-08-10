% Plots trajectories for anlges and velocities
%   phase portrait for q_t vs dq_t
%
% Input:
%       t: time(s) for the corresponding gait in x
%       x: [q1, q2, q3, dq1, dq2, dq3]
%
% angles are in radians, velocities are in rad/s
%
function plot_trajectories(t,x)

% Time is always 0 at the begining of an ODE45 solution, so it is the most
% consistent way to seperate phases and steps
%
% Find index when time = 0:
ind0 = find(t == 0);
% Number of steps taken:
m = length(ind0);

% Add extra element to index to help with seperation
ind0(length(ind0)+1) = length(t)+1;

t_tot = [];

for i = 1:m
    
    if isempty(t_tot)
        t_end = 0;
    else
        t_end = t_tot(end);
    end
    
    t_tot = [t_tot; t_end+t(ind0(i):ind0(i+1)-1)];
    
end

% Plot joint angles:
figure()
subplot(2,1,1)
plot(t_tot,x(:,1)*180/pi), hold on
plot(t_tot,x(:,2)*180/pi)
plot(t_tot,x(:,3)*180/pi)

hold off, grid on
title('Joint angles')
xlabel('t (s)')
ylabel('q (deg)')
legend('q_1','q_2','q_3')

% Plot joint velocities:
subplot(2,1,2)
plot(t_tot,x(:,4)*180/pi), hold on
plot(t_tot,x(:,5)*180/pi)
plot(t_tot,x(:,6)*180/pi)

hold off, grid on
title('Joint velocities')
xlabel('t (s)')
ylabel('dq (deg/s)')
legend('dq_1','dq_2','dq_3')

% Plot phase portrait for q1 vs dq1
figure()
plot(x(:,1)*180/pi,x(:,4)*180/pi)
grid on
title('Phase portrait of q_1 and dq_1')
xlabel('q_1 (deg)')
ylabel('dq_1 (deg/s)')
end