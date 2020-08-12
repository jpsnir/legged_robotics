load('state_variable_mt_10_l_50cm.mat');
z_50cm = [z_tot(:,1),z_tot(:,2)];
load('state_variable_mt_10_l_25cm.mat');
z_25cm = [z_tot(:,1),z_tot(:,2)];
load('state_variable_mt_10_l_10cm.mat');
z_10cm = [z_tot(:,1),z_tot(:,2)];


figure(1);
plot(z_50cm(:,1),z_50cm(:,2),...
    z_25cm(:,1),z_25cm(:,2),...
    z_10cm(:,1),z_10cm(:,2));
xlabel('q1');
ylabel('q1_d');
legend('50 cm torso','25 cm torso','10 cm torso');

load('state_variable_mt_10_l_25cm.mat');
z_10kg = [z_tot(:,1),z_tot(:,2)];
load('state_variable_mt_5_l_25cm.mat');
z_5kg = [z_tot(:,1),z_tot(:,2)];
load('state_variable_mt_15_l_25cm.mat');
z_15kg = [z_tot(:,1),z_tot(:,2)];
load('state_variable_mt_1_l_25cm.mat');
z_1kg = [z_tot(:,1),z_tot(:,2)];

figure(2);
plot(z_1kg(:,1),z_1kg(:,2),...
    z_5kg(:,1),z_5kg(:,2),...
    z_10kg(:,1),z_10kg(:,2),...
    z_15kg(:,1),z_15kg(:,2));
xlabel('q1');
ylabel('q1_d');
legend('1 kg torso','5 kg torso','10 kg torso','15 kg torso');
