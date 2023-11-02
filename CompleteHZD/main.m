% ---------------------------------------------------
% Team:       Andrew Lessieur, Jagatpreet Nir,
%             Edward Wiley, Stephen Hagen
% 
% Purpose:    Provide a clear starting point for running our project code
%             as well as give user the choice to run parts or all of code
% 
% Calls:        
% 
%    Step 1:  generate_functions.m [Optional, unless Autogen is empty]
% 
%    Step 2:  OptimizeClean.m [Optional if you have f already]
% 
%    Step 3:  simZD2.m [Optional]
% 
%    Step 4:  simFull.m [Optional]
% 
% Notes:
% 
%         1: generate_functions.m assumes CCW positive for angles
%            q1 is zero from vertical axis
%            q2, q3 are zero from stance leg
% 
%         2: OptimizeClean.m is parameterized by theta,
%            which is related to step length by
%            step_length = 2*r*cos(theta)
% 
%         3: simZD2 now takes input flag, bez, to denote
%            if you want to plot Bezier Curves or not
%            Plots have coefficients in [deg]
% 
%         4: sim_and_plot_full_dynamics.m calls
%            func_feedback.m to compute the required input.
%            That file contains PD gains. Do tuning there.
% 
% ---------------------------------------------------

% Include util and autogen folders
set_path

close all
clear all %#ok

% ---------------------------------------------------
% Choose Scripts to Run
% ---------------------------------------------------

% Set to 0 if you don't want to run generate4.m
runGen = 0;

% Set to 0 if you don't want to run the optimizer
% WARNING: Only comment out if you have f already
runOpt = 1;

% Change to 0 if you don't want to run simZD.m
runSimZ = 1;
% Change to 0 if you don't want to plot Bezier Curves
bez = 1;

% Change to 0 if you don't want to run simFull.m
runSimF = 1;

% ---------------------------------------------------
% Check if Autogen is Filled
% ---------------------------------------------------

cd autogen\
MyFolderInfo = dir;
L = length(MyFolderInfo);
cd ..
if L < 9
    disp('Running generate')
    generate4;
end


% ---------------------------------------------------
% Run Generate Anyway
% ---------------------------------------------------

if L < 9
    disp('Generate has been run')
else
    if runGen == 1
        disp('Running generate again')
        generate_functions
    end
end

% ---------------------------------------------------
% Optimization
% ---------------------------------------------------

if runOpt == 1
    % Optimization Parameter Input [deg or deg/s]
    stepLen = 0.7;
    theta = acosd(stepLen/2);
    q1d = -170;
    q3f = 170;
    q3df = 5;
    a3 = 5;
    g3 = 155;
    disp('Running Optimizer')
    OptimizeClean
end

% ---------------------------------------------------
% Zero Dynamics Simulation
% ---------------------------------------------------

if (runSimZ==1)
    if ~exist('f','var')% checks if f is in workspace
        error('f is not defined, cannot run simZD')
    else
        disp('Running Zero Dynamics Simulation')
        simZD2
    end
end

% ---------------------------------------------------
% Full Dynamics Simulation
% ---------------------------------------------------

if (runSimF==1)
    if ~exist('f','var') % checks if f is in workspace
        error('f is not defined, cannot run simFull')
    else
        disp('Running Full Dynamics Simulation')
        sim_and_plot_full_dynamics
    end
end