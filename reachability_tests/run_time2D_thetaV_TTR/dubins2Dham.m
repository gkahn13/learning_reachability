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
%   |v| <= vMax
% Somil Bansal, 2016-10-13

checkStructureFields(schemeData, 'grid', 'vMax');

g = schemeData.grid;
vMax = schemeData.vMax;

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
  v = -vMax;
  
  if isfield(schemeData, 'dMode')
    dMax = schemeData.dMax;
    v = v + dMax;
  end
    
else
  error('Unknown uMode! Only ''min'' mode has been implemented so far.')
end

%% Hamiltonian control terms
% Speed control
derivNorm = sqrt(deriv{1}.^2 + deriv{2}.^2);
hamValue = v*derivNorm;

%% Backward or forward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
else
  error('tMode must be ''backward''!')
end
end
