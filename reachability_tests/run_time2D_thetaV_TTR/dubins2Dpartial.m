function alpha = dubins2Dpartial(t, data, derivMin, derivMax, schemeData, dim)
% Inputs:
%   schemeData - problem parameters
%     .grid: grid structure
%     .vXrange: speed range of vehicle in x-direction
%     .vYrange: speed range of vehicle in y-direction
%
% Dynamics:
%   \dot x      = v_x
%   \dot y      = v_y
% Somil Bansal, 2016-10-13

checkStructureFields(schemeData, 'grid', 'vMax');

g = schemeData.grid;
vMax = schemeData.vMax;

if isfield(schemeData, 'dMode')
  dMax = schemeData.dMax;
  vMax = vMax + dMax;
end

switch dim
  case 1
    % Control
    alpha = vMax;
    
  case 2
    % Control
    alpha = vMax; 
    
end
end