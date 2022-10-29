function states = ctrect(states, varargin)
% CTRECT A constant turn-rate rectangular target motion model
%   states = CTRECT(states) calculates the state at the next time-step
%   based on the current state, assuming dT = 1 second.
%
%   state = CTRECT(state, dt) calculates the state at the next time-step
%   based on the current state and the time interval, dt.
%
%   state = CTRECT(state, w, dt) calculates the state at the next
%   time-step based on the current state, the noise w, and the time
%   interval, dt.
%
%   state can be defined as a vector or matrix. If specified as a vector,
%   the orientation of the state vector can be either column or row vector,
%   and the output will match the input's orientation. The function uses
%   the speed and heading to describe the state according to the
%   following convention:
%
%   state = [x;y;s;theta;omega;L;W]
%
%   where:
%   x and y are positions in meters
%   s is the speed in meters/second
%   theta is the orientation or heading of the rectangle in degrees
%   omega is the turn-rate in degrees/second
%   L and W are length and width of the rectangle respectively in meters
%
%   If state is specified as a matrix, it must be concatenated along
%   columns, where each column represents a state defined according to the
%   convention above.
%
%   If specified, w must be a scalar or a 2-by-N matrix, where N is the
%   number of state columns. If specified as a scalar, it will be expanded
%   to a 2-by-N matrix. The first row specifies the process noise in
%   acceleration (meters/second^2) and the second row specifies the process
%   noise in yaw acceleration (degrees/second^2)
%
%   Example 1: predict a constant turn rate rectangular state
%   ---------------------------------------------------------
%   % Define a state vector
%   state = [1;2;2;30;1;4.7;1.8];
%
%   % Predict the state assuming dt = 1 second
%   state = CTRECT(state);
%
%   % Predict the state given dt = 0.1 second
%   state = CTRECT(state, 0.1);
%
%   % Predict the state using a process noise inputs and dt = 0.1 second
%   state = CTRECT(state, [0;0], 0.1);
%
%   % Example 2: predict multiple constant turn-rate rectangular states
%   --------------------------------------------------------------------
%   % Define a state matrix
%   states = [1 3 4;-1 2 10;5 3 1.3;1 1.3 2.1;30 0 -30;4.7 3.4 4.5;1.8 2 3];
%
%   % Predict the state assuming dt = 1 second
%   states = ctrect(states);
%
%   % Predict the state given dt = 0.1 second
%   states = ctrect(states,0.1);
%
%   % Predict the state using process noise inputs and dt = 0.1 second
%   states = ctrect(states, 0.1*randn(2,3), 0.1);
%   
%   See also: gmphd, ctrectjac, ctrectmeas, ctrectmeasjac, initctrectgmphd

%   Copyright 2019 The MathWorks, Inc.

%#codegen

narginchk(1,3)

% Validate state input
validateattributes(states, {'single','double'},...
    {'real', 'finite', '2d', 'nonsparse'}, 'ctrect', 'state', 1);

% Check for row or column Orientation
[rowSize,colSize] = size(states);
isRowOrient = colSize == 7;
isColOrient = rowSize == 7;

% Only column orientation is supported for vectorized state
cond = (~isRowOrient || ~isvector(states)) && ~isColOrient;
coder.internal.errorIf(cond, 'fusion:ctrect:incorrectStateSizeWithInfo');

if ~isColOrient
    stateCol = states';
else
    stateCol = states;
end

classToUse = class(states);
numCols = size(stateCol,2);

% If there are 3 input arguments, it means that the 2nd one (first
% varargin) is w
if nargin==3 || (nargin==2 && ~isscalar(varargin{1}))
    validateattributes(varargin{1},{'double','single'},...
        {'real','finite','nonsparse'}, mfilename, 'w', 2);
    w = cast(varargin{1},classToUse);
    if isscalar(w)
        wcol = w * ones(2,numCols,classToUse);
    else
        wcol = w;
    end
    coder.internal.assert(isequal(size(wcol),[2 numCols]),'shared_tracking:motion:incorrectNoiseDim','w',2,numCols);
else
    wcol = zeros(2,numCols,classToUse);
end

% If dt is not given, assume dt = 1
if nargin==1 || (nargin==2 && ~isscalar(varargin{1}))
    dT = ones(1, 1, 'like', stateCol);
else % dt is always at the end of varargin
    validateattributes(varargin{end}, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonsparse'}, 'ctrect', 'dt', nargin);
    dT = varargin{end};
end

x = stateCol(1,:);
y = stateCol(2,:);
s = stateCol(3,:);
theta = deg2rad(stateCol(4,:));
omega = deg2rad(stateCol(5,:));

% Prevent division by zero omega.
omega = max(abs(omega),eps(classToUse));
invIdx = stateCol(5,:) < 0 & omega > 0;
omega(invIdx) = -omega(invIdx);

% Pre-compute values
sinTheta = sin(theta);
cosTheta = cos(theta);
sinwt = sin(omega*dT);
coswt = cos(omega*dT);

% Process noise in x, y and yaw
ws = wcol(1,:);
wx = ws.*cosTheta;
wy = ws.*sinTheta;
wOmega = deg2rad(wcol(2,:));

% State update equations
xK = x + s./omega.*(cosTheta.*sinwt - sinTheta.*(1 - coswt)) + wx*dT^2/2;
yK = y + s./omega.*(sinTheta.*sinwt + cosTheta.*(1 - coswt)) + wy*dT^2/2;
thetaK = (theta + omega*dT + wOmega*dT^2/2);
sK = s + ws*dT;
omegaK = omega + wOmega*dT;

% Put back in states
stateCol(1,:) = xK;
stateCol(2,:) = yK;
stateCol(3,:) = sK;
stateCol(4,:) = fusion.internal.UnitConversions.interval(rad2deg(thetaK),cast([-180 180],classToUse));
stateCol(5,:) = rad2deg(omegaK);

if ~isColOrient
    states = stateCol';
else
    states = stateCol;
end

end