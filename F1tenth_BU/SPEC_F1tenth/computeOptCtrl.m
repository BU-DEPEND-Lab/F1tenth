function [traj_u] = computeOptCtrl(g, data, tau, dynSys, extraArgs)
if nargin < 5
  extraArgs = [];
end

% Default parameters
uMode = 'min';
subSamples = 4;

if isfield(extraArgs, 'uMode')
  uMode = extraArgs.uMode;
end

if isfield(extraArgs, 'subSamples')
  subSamples = extraArgs.subSamples;
end

clns = repmat({':'}, 1, g.dim);

if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

% Time parameters
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/subSamples;
% maxIter = 1.25*tauLength;

% Initialize control sequences
traj_u = nan(dynSys.nu, subSamples);
tEarliest = 1;

% Determine the earliest time that the current state is in the reachable set
% Binary search
upper = tauLength;
lower = tEarliest;

tEarliest = find_earliest_BRS_ind(g, data, dynSys.x, upper, lower);

% BRS at current time
BRS_at_t = data(clns{:}, tEarliest);


  
% Update trajectory
Deriv = computeGradients(g, BRS_at_t);
for j = 1:subSamples
    deriv = eval_u(g, Deriv, dynSys.x);
    u = dynSys.optCtrl(tau(tEarliest), dynSys.x, deriv, uMode);
    dynSys.updateState(u, dtSmall, dynSys.x);
    traj_u(:,j) = u;
end
end