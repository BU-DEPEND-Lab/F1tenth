function [traj, traj_u, traj_tau, failure] = compute_traj(xinit, g, data, tau2, visual)
%% Input Parsing
if nargin == 0
    xinit = [-4.5, 0, 0];
    visual = true;
elseif nargin == 1
    visual = true;
end

%% problem parameters

% input bounds
speed = 0.5;
wMax = 0.7;
% do dStep1 here

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here


%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = DubinsCar([0, 0, 0], wMax, speed); %do dStep3 here

%% Compute trajectory
if visual
    disp('Press any button to continue')
    pause
end

%check if this initial state is in the BRS/BRT
%value = eval_u(g, data, x)
value = eval_u(g, data(:,:,:,end),xinit);

if value <= 0 %if initial state is in BRS/BRT
    failure = false;
    % find optimal trajectory

    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = visual; %show plot
    TrajextraArgs.fig_num = 2; %figure number

    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 

    % subsamples for control
    TrajextraArgs.subSamples = 1;

    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);

    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_u, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
else
    failure = true;
    traj = 0;
    traj_u = 0;
    traj_tau = 0;
    % error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
end
end