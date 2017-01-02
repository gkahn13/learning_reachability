clear; clc; 
tic
%% Grid
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];    % Upper corner of computation domain
N = [41; 41];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
data0 = shapeCylinder(g, [], [4; 4], R);
% tempMat = ones(g.shape);
% target_index = find(data0 <= 0);
% tempMat(target_index) = -1;
% data0 = tempMat;
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 8;
dt = 0.01;
tau = t0:dt:tMax;
% tau = [t0 tMax];
% If intermediate results are not needed, use tau = [t0 tMax];

%% problem parameters
vMax = 1;
dMax = 0.1;

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!

% Dynamical system parameters
schemeData.grid = g;
schemeData.hamFunc = @dubins2Dham;
schemeData.partialFunc = @dubins2Dpartial;
schemeData.vMax = vMax;
schemeData.uMode = 'min';
schemeData.tMode = 'backward';
schemeData.dMode = 'max';
schemeData.dMax = dMax;

% x(:,1) = [-3.1;-3];
%% Define obstacles
% Obstacles around the boundary
obstacles = -shapeRectangleByCenter(g, [0; 0], 2*(max(grid_max)-max(g.dx)));

% Other obstacles
obstacles_grid1 = shapeCylinder(g, [], [1.5; 1.5], 0.5*R);
obstacles_grid2 = shapeCylinder(g, [], [-1.5; -1.5], 0.5*R);
obstacles_grid = shapeUnion(obstacles_grid1, obstacles_grid2);
% tempMat = ones(g.shape);
% obs_index = find(obstacles_grid <= 0);
% tempMat(obs_index) = -1;
% obstacles_grid = tempMat;
obstacles = shapeUnion(obstacles, obstacles_grid);

extraArgs.obstacles = obstacles;
% extraArgs.stopInit = x(:,1);

%% Start reachability computation
targets = data0;
extraArgs.targets = targets;
extraArgs.visualize = true;

numPlots = 4;
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots / spC);

[data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
toc

%% Let's do a quick simulation
x(:,1) = [3; -4];
TTR = TD2TTR(g, data, tau);

for i=1:length(tau)
  
  % Find the correct level set index
  tau_current(i) = eval_u(g, TTR, x(:,i));
  LSindex = max(max(find(tau < tau_current(i))) + 1, length(tau));
  
  % Compute co-state
  P = computeGradients(g, data(:,:,LSindex));
%   pVis = max(abs(P{1}), abs(P{2})); 
%   visSetIm(g, pVis, 'b', 0.001, [], false);
  
  p(:,i) = calculateCostate(g, P, x(:,i));
  
  % Compute optimal control
  v_opt(:,i) = -vMax*p(:,i)/norm(p(:,i));
  if max(abs(p(:,i))) < 0.01
    rotMat = [0, -1 ; 1, 0];
%      v_opt(:, i) = rotMat*v_opt(:, i);
  end
  
  % Update state 
  x(1,i+1) = x(1,i) + dt*v_opt(1, i);
  x(2,i+1) = x(2,i) + dt*v_opt(2, i);
  
  hold on;
  plot(x(1,i), x(2,i), 'k.');
  
  % Stop if we have reached the target
  initValue = eval_u(g, data(:,:,1), x(:,i+1));
  if initValue <= 0
    break;
  end
end
