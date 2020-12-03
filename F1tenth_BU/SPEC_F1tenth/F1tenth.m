function [traj, traj_u, traj_tau, empty_flag] = F1tenth(barrel_size, dis_barrel, dis_wall, xinit, visual)
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

dis_target = 0;

%% Input Parsing
if nargin == 0
    barrel_size = 0.5;
    dis_barrel = 0;
    dis_wall = 0;
    xinit = [-4.5, -4, 0];
    visual = true;
elseif nargin == 1
    dis_barrel = 0;
    dis_wall = 0;
    xinit = [-4.5, -4, 0];
    visual = true;
elseif nargin == 3
    dis_wall = 0;
    xinit = [-4.5, -4, 0];
    visual = true;
elseif nargin == 4
    xinit = [-4.5, -4, 0];
    visual = true;
elseif nargin == 5
    visual = false;
end

%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [-6; -6; -pi]; % Lower corner of computation domain
grid_max = [6; 10; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% data0 = shapeCylinder(g, 3, [4; -4; 0], R);
data0 = shapeRectangleByCenter(g, [-3; -1; 0], [R-dis_target; R-dis_target; 1]);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 60;
dt = 0.1;
tau = t0:dt:tMax;

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

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here

%% initial set
R = 0.5;
initial_points = rand(100, dCar.nx);
initial_points(:,1) = initial_points(:,1)*2*R - 4.5;
initial_points(:,2) = initial_points(:,2)*2*R - 4;
initial_points(:,3) = initial_points(:,3)*2*1 - 1;

%% additive random noise
%do Step8 here
%HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here
% Barrel
obstacles_barrel = shapeRectangleByCenter(g, [-1; -4; 0], [barrel_size+dis_barrel; barrel_size+dis_barrel; 1]);
% Right Walls
% obstacles_right_wall = shapeRectangleByCenter(g, [-0.086; -2.591; 0], [5.229*2+dis_right; 6+dis_right; 1]);
% obstacles_right_wall = shapeComplement(obstacles_right_wall);
% Hyperplane version
obstacles_right_wall1 = shapeHyperplane(g, [0; 1; 0], [5.2; -5.4+dis_wall; 0]);
obstacles_right_wall2 = shapeHyperplane(g, [1; 0; 0], [-5.2+dis_wall; -5.4; 0]);
obstacles_right_wall3 = shapeHyperplane(g, [-1; 0; 0], [5.2-dis_wall; -5.4; 0]);
obstacles_right_wall4 = shapeHyperplaneByPoints(g, [5.2-dis_wall 3.3-dis_wall 5.2-dis_wall; 6-dis_wall 9-dis_wall 6-dis_wall; 0 0 1].');
obstacles_right_wall5 = shapeHyperplane(g, [0; -1; 0], [3.3; 9-dis_wall; 0]);
obstacles_right_wall = shapeUnion(obstacles_right_wall1, obstacles_right_wall2);
obstacles_right_wall = shapeUnion(obstacles_right_wall, obstacles_right_wall3);
obstacles_right_wall = shapeUnion(obstacles_right_wall, obstacles_right_wall4);
obstacles_right_wall = shapeUnion(obstacles_right_wall, obstacles_right_wall5);
% Left Wall
% obstacles_left_wall = shapeRectangleByCenter(g, [0.397; 0.434; 0], [5.266+dis_left; 6+dis_left; 1]);
% Hyperplane version
obstacles_left_wall1 = shapeHyperplane(g, [0; -1; 0], [2; -2.3-dis_wall; 0]);
obstacles_left_wall2 = shapeHyperplane(g, [0; 1; 0], [1.2; -1.4+dis_wall; 0]);
obstacles_left_wall3 = shapeHyperplane(g, [-1; 0; 0], [-5.2; -5.4; 0]);
obstacles_left_wall4 = shapeHyperplane(g, [1; 0; 0], [2+dis_wall; -2.3; 0]);

obstacles_left_wall5 = shapeHyperplane(g, [-1; 0; 0], [-2.05-dis_wall; 5.2; 0]);
obstacles_left_wall6 = shapeHyperplane(g, [0; -1; 0], [1.2; 5.2-dis_wall; 0]);
obstacles_left_wall7 = shapeHyperplane(g, [0; 1; 0], [2; 6.1+dis_wall; 0]);

obstacles_left_wall8 = shapeHyperplane(g, [-1; 0; 0], [1.2-dis_wall; -1.4; 0]);

obstacles_left_wall = shapeIntersection(obstacles_left_wall1, obstacles_left_wall2);
obstacles_left_wall = shapeIntersection(obstacles_left_wall, obstacles_left_wall3);
obstacles_left_wall = shapeIntersection(obstacles_left_wall, obstacles_left_wall4);

obstacles_left_wall_second = shapeIntersection(obstacles_left_wall4, obstacles_left_wall5);
obstacles_left_wall_second = shapeIntersection(obstacles_left_wall_second, obstacles_left_wall6);
obstacles_left_wall_second = shapeIntersection(obstacles_left_wall_second, obstacles_left_wall7);

obstacles_left_wall_third = shapeIntersection(obstacles_left_wall4, obstacles_left_wall8);
obstacles_left_wall_third = shapeIntersection(obstacles_left_wall_third, obstacles_left_wall1);
obstacles_left_wall_third = shapeIntersection(obstacles_left_wall_third, obstacles_left_wall7);
obstacles_left_wall_fourth = shapeUnion(obstacles_left_wall, obstacles_left_wall_second);
obstacles_left_wall_fourth = shapeUnion(obstacles_left_wall_fourth, obstacles_left_wall_third);

% Triangle wall
obstacles_left_tri1 = shapeHyperplaneByPoints(g, [2 4.9 2; -2.3 2, -2.3; 0 0 1].',[5.2;-5.4;0]);
obstacles_left_tri2 = shapeHyperplaneByPoints(g, [4.9 2 4.9; 2 6.1 2; 0 0 1].',[5.2;6;0]);
obstacles_left_tri_tmp = shapeIntersection(obstacles_left_tri1, obstacles_left_tri2);
obstacles_left_wall9 = shapeHyperplane(g, [-1; 0; 0], [2+dis_wall; -2.3; 0]);
obstacles_left_tri = shapeIntersection(obstacles_left_tri_tmp, obstacles_left_wall9);

obstacles_left_wall = shapeUnion(obstacles_left_wall_fourth, obstacles_left_tri);

obstacles_wall1 = shapeHyperplane(g, [1; 0; 0], [-2+dis_wall; 2.1; 0]);
obstacles_wall2 = shapeHyperplane(g, [0; -1; 0], [-2; 1.2-dis_wall; 0]);
obstacles_wall3 = shapeHyperplane(g, [0; 1; 0], [-2; 2.1+dis_wall; 0]);
obstacles_wall = shapeIntersection(obstacles_wall1, obstacles_wall2);
obstacles_wall = shapeIntersection(obstacles_wall, obstacles_wall3);

% Shape Union
obstacles = shapeUnion(obstacles_barrel, obstacles_left_wall);
obstacles = shapeUnion(obstacles, obstacles_wall);
obstacles = shapeUnion(obstacles, obstacles_right_wall);
HJIextraArgs.obstacles = obstacles;
%% Compute value function

HJIextraArgs.visualize = visual; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);

empty_flag = all(eval_u(g,data(:,:,:,end),initial_points) < 0);

%% Compute optimal trajectory from some initial state
if compTraj
  if visual
      pause
  end
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
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
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end
