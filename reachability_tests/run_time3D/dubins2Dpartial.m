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

checkStructureFields(schemeData, 'grid', 'vXrange', 'vYrange');

g = schemeData.grid;
vXMax = max(schemeData.vXrange);
vYMax = max(schemeData.vYrange);

if isfield(schemeData, 'dMode')
  dXrange = schemeData.dXrange;
  dYrange = schemeData.dYrange;
  dXMax = dXrange(2);
  dYMax = dYrange(2);

  vXMax = vXMax + dXMax;
  vYMax = vYMax + dYMax;
end

switch dim
  case 1
    % Control
    alpha = vXMax;
    
  case 2
    % Control
    alpha = vYMax; 
    
end
end