function [traj_u, failure] = compute_ctrl(x, g, data, dataTraj, tau2, dCar, visual)

%% Input Parsing
if nargin == 0
    x = [-4.5, 0, 0];
    visual = true;
elseif nargin == 1
    visual = false;
end

%% problem parameters

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here

%% Compute trajectory
if visual
    disp('Press any button to continue')
    pause
end

%check if this initial state is in the BRS/BRT
%value = eval_u(g, data, x)
value = eval_u(g, data(:,:,:,end),x);

if value <= 0 %if initial state is in BRS/BRT
    failure = false;
    % find optimal trajectory

    dCar.x = x; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = visual; %show plot
    TrajextraArgs.fig_num = 2; %figure number

    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0];

    % subsamples for control
    TrajextraArgs.subSamples = 1;

    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj_u] = ...
      computeOptCtrl(g, dataTraj, tau2, dCar, TrajextraArgs);
else
    failure = true;
    traj_u = 0;
    % error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
end
end