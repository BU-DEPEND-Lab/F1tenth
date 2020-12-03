function [g, data, dataTraj, tau2, dCar, empty_flag] = Grid_data(barrel_size, dis_barrel, dis_wall, dis_target, visual)
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

%% Input Parsing
if nargin == 0
    barrel_size = 0.5;
    dis_barrel = 0;
    dis_wall = 0;
    dis_target = 0;
    visual = true;
elseif nargin == 1
    dis_barrel = 0;
    dis_wall = 0;
    dis_target = 0;
    visual = false;
elseif nargin == 3
    dis_wall = 0;
    dis_target = 0;
    visual = false;
elseif nargin == 4
    dis_target = 0;
    visual = false;
elseif nargin == 5
    visual = false;
end

%% Grid
grid_min = [-6; -6; -pi]; % Lower corner of computation domain
grid_max = [6; 6; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% data0 = shapeCylinder(g, 3, [4; -4; 0], R);
data0 = shapeRectangleByCenter(g, [4; -4; 0], [R-dis_target; R-dis_target; 1]);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 20;
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
obstacles_right_wall1 = shapeHyperplane(g, [0; 1; 0], [5.142; -5.591+dis_wall; 0]);
obstacles_right_wall2 = shapeHyperplane(g, [1; 0; 0], [-5.815+dis_wall; -5.591; 0]);
obstacles_right_wall3 = shapeHyperplane(g, [-1; 0; 0], [5.142-dis_wall; -5.591; 0]);
obstacles_right_wall = shapeUnion(obstacles_right_wall1, obstacles_right_wall2);
obstacles_right_wall = shapeUnion(obstacles_right_wall, obstacles_right_wall3);
% Left Wall
% obstacles_left_wall = shapeRectangleByCenter(g, [0.397; 0.434; 0], [5.266+dis_left; 6+dis_left; 1]);
% Hyperplane version
obstacles_left_wall1 = shapeHyperplane(g, [0; -1; 0], [3.030; -2.374-dis_wall; 0]);
obstacles_left_wall2 = shapeHyperplane(g, [-1; 0; 0], [-2.236-dis_wall; -2.374; 0]);
obstacles_left_wall3 = shapeHyperplane(g, [1; 0; 0], [3.030+dis_wall; -2.374; 0]);
obstacles_left_wall = shapeIntersection(obstacles_left_wall1, obstacles_left_wall2);
obstacles_left_wall = shapeIntersection(obstacles_left_wall, obstacles_left_wall3);
% Shape Union
obstacles = shapeUnion(obstacles_barrel, obstacles_left_wall);
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

%flip data time points so we start from the beginning of time
dataTraj = flip(data,4);

empty_flag = all(eval_u(g,data(:,:,:,end),initial_points) < 0);
end