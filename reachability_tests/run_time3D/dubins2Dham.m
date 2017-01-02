function hamValue = dubins2Dham(t, data, deriv, schemeData)
% hamValue = dubins2Dham(deriv, schemeData)
%   Hamiltonian function for Dubins car used with the level set toolbox
%
% Inputs:
%   schemeData - problem parameters
%     .grid:   grid structure
%     .vXrange: speed range of vehicle in x-direction
%     .vYrange: speed range of vehicle in y-direction
%     .uMode:  'min' or 'max' (defaults to 'min')
%     .tMode: 'backward' or 'forward'
%
% Dynamics:
%   \dot x      = v_x
%   \dot y      = v_y
% Somil Bansal, 2016-10-13

checkStructureFields(schemeData, 'grid', 'vXrange', 'vYrange');

g = schemeData.grid;

%% Speed range
vXrange = schemeData.vXrange;
vYrange = schemeData.vYrange;
if isscalar(vXrange)
  vXrange = [-vXrange vXrange];
end
if isscalar(vYrange)
  vYrange = [-vYrange vYrange];
end
vXMin = vXrange(1);
vXMax = vXrange(2);
vYMin = vYrange(1);
vYMax = vYrange(2);

%% Defaults: min over control, backward reachable set
if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

%% Modify Hamiltonian control terms based on uMode
if strcmp(schemeData.uMode, 'min')
  % the speed depending on the determinant term (terms multiplying v) is positive
  % or negative
  vX = vXMax*(deriv{1} <= 0) + vXMin*(deriv{1} > 0);
  vY = vYMax*(deriv{2} <= 0) + vYMin*(deriv{2} > 0);
  
  if isfield(schemeData, 'dMode')
    dXrange = schemeData.dXrange;
    dYrange = schemeData.dYrange;
    dXMin = dXrange(1);
    dXMax = dXrange(2);
    dYMin = dYrange(1);
    dYMax = dYrange(2);
    
    dX = dXMin*(deriv{1} <= 0) + dXMax*(deriv{1} > 0);
    dY = dYMin*(deriv{2} <= 0) + dYMax*(deriv{2} > 0);
    
    vX = vX + dX;
    vY = vY + dY;
  end
    
else
  error('Unknown uMode! Only ''min'' mode has been implemented so far.')
end

%% Hamiltonian control terms
% Speed control
hamValue = vX.* deriv{1} + vY.* deriv{2};

%% Backward or forward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
else
  error('tMode must be ''backward''!')
end
end
