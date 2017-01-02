clear; clc; close all;
tic
%% Grid
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];    % Upper corner of computation domain
N = [101; 101];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
data0 = shapeCylinder(g, [], [4; 4], 0.05*R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 10;
dt = 0.025;
tau = t0:dt:tMax;
% tau = [t0 tMax];
% If intermediate results are not needed, use tau = [t0 tMax];

%% problem parameters
vX = [-1, 1];
vY = [-1, 1];

dX = [-0.1, 0.1];
dY = [-0.1, 0.1];

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!

% Dynamical system parameters
schemeData.grid = g;
schemeData.hamFunc = @dubins2Dham;
schemeData.partialFunc = @dubins2Dpartial;
schemeData.vXrange = vX;
schemeData.vYrange = vY;
schemeData.uMode = 'min';
schemeData.tMode = 'backward';
schemeData.dMode = 'max';
schemeData.dXrange = dX;
schemeData.dYrange = dY;

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
x(:,1) = [3;-4];
for i=1:length(tau)
  % Compute co-state
  P = computeGradients(g, data(:,:,end-i+1));
  p(:,i) = calculateCostate(g, P, x(:,i));
  
  % Compute optimal control
  vXMax = max(vX);
  vYMax = max(vY);
  vXMin = min(vX);
  vYMin = min(vY);
  vX_opt(i) = vXMax*(p(1,i) <= 0) + vXMin*(p(1,i) > 0);
  vY_opt(i) = vYMax*(p(2,i) <= 0) + vYMin*(p(2,i) > 0);
  if max(abs(p(:,i))) < 0.01
%     vX_opt(i) = -vX_opt(i);
  end
  
  % Update state 
  x(1,i+1) = x(1,i) + dt*vX_opt(i);
  x(2,i+1) = x(2,i) + dt*vY_opt(i);
  
  hold on;
  plot(x(1,i), x(2,i), 'k.');
  
  % Stop if we have reached the target
  initValue = eval_u(g, data(:,:,1), x(:,i+1));
  if initValue <= 0
    break;
  end
end
