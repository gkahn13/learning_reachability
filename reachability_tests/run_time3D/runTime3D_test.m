clear; clc; close all;
tic
%% Grid
grid_min = [-5; -5; -pi]; % Lower corner of computation domain
grid_max = [5; 5; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 8;
dt = 2;
tau = [t0:dt:tMax];
% If intermediate results are not needed, use tau = [t0 tMax];

%% problem parameters
speed = 1;
wMax = 1;

%% Pack problem parameters
% Dynamical system parameters
dCar = DubinsCar([0, 0, 0], wMax, speed);
schemeData.grid = g;
schemeData.dynSys = dCar;

%% Define obstacles
obstacles_grid1 = shapeCylinder(g, 3, [1.5; 1.5; 0], 0.5*R);
obstacles_grid2 = shapeCylinder(g, 3, [-1.5; -1.5; 0], 0.5*R);
obstacles_grid = shapeUnion(obstacles_grid1, obstacles_grid2);
% obstacles = ones(g.shape);
% obs_index = find(obstacles_grid <= 0);
% obstacles(obs_index) = -1;
obstacles = obstacles_grid;

extraArgs.obstacles = obstacles;
% extraArgs.stopInit = x(:,1);

targets = data0;
extraArgs.targets = targets;
extraArgs.visualize = true;

numPlots = 4;
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots / spC);

[data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
toc

%% Let's do a quick simulation
x(:,1) = [-3; -3; 0];
for i=1:length(tau)
  % Compute co-state
  P = computeGradients(g, data(:,:,end-i+1));
  p(:,i) = calculateCostate(g, P, x(:,i));
  
  % Compute optimal control
  vXMax = max(vX);LQR
  vYMax = max(vY);
  vXMin = min(vX);
  vYMin = min(vY);
  vX_opt(i) = vXMax*(p(1,i) <= 0) + vXMin*(p(1,i) > 0);
  vY_opt(i) = vYMax*(p(2,i) <= 0) + vYMin*(p(2,i) > 0);
  
  % Update state 
  x(1,i+1) = x(1,i) + dt*vX_opt(i);
  x(2,i+1) = x(2,i) + dt*vY_opt(i);
  
  hold on;
  plot(x(1,i), x(2,i), 'k.');
  keyboard;
end
