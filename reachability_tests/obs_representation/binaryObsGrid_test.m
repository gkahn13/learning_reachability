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
tMax = 2;
dt = 0.025;
tau = t0:dt:tMax;
% If intermediate results are not needed, use tau = [t0 tMax];

%% problem parameters
speed = 1;
wMax = 1;

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!

% Dynamical system parameters
dCar = DubinsCar([0, 0, 0], wMax, speed);
schemeData.grid = g;
schemeData.dynSys = dCar;

%% Test using single obstacle
obstacles_grid1 = shapeCylinder(g, 3, [1.5; 1.5; 0], 0.75*R);
obstacles_grid2 = shapeCylinder(g, 3, [-1.5; -1.5; 0], 0.75*R);
obstacles_grid = shapeUnion(obstacles_grid1, obstacles_grid2);
obstacles = ones(g.shape);
obs_index = find(obstacles_grid <= 0);
obstacles(obs_index) = -1;

extraArgs.obstacles = obstacles;

targets = data0;
extraArgs.targets = targets;
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1,1,0];
extraArgs.plotData.projpt = 0;

numPlots = 4;
spC = ceil(sqrt(numPlots));
spR = ceil(numPlots / spC);

[data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

% Visualize
figure;
for i = 1:numPlots
  subplot(spR, spC, i)
  ind = ceil(i * length(tau) / numPlots);
  visualizeLevelSet(g, data(:,:,:,ind), 'surface', 0, ...
    ['TD value function, t = ' num2str(tau(ind))]);
  axis(g.axis)
  camlight left
  camlight right
  drawnow
end