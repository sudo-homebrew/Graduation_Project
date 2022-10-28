function dfdx = singerjac(state, varargin)
% SINGERJAC  Jacobian of Singer acceleration motion model.
%   The trackingEKF object allows you to specify the
%   StateTransitionJacobianFcn property. SINGERJAC provides the Jacobian
%   of the Singer acceleration state transition function, singer, with
%   respect to the state.
%
%   dfdx = SINGERJAC(state) calculates the Jacobian matrix of the
%   Singer acceleration motion model with respect to the state vector. It
%   is assumed that dt = 1 second.
%
%   dfdx = SINGERJAC(state, dt) allows you to specify a time interval, dt.
%
%   dfdx = SINGERJAC(state, dt, tau) additionally, allows you the specify
%   the target maneuver time constant, tau. If not specified, it is assumed
%   to be 20 seconds.
%
%   The dfdx Jacobian is an n-by-n matrix, where n is the number of states
%   in the state vector. If rm = exp(-dt/tau), then
%   In a 1-D motion, the Jacobian is:
%       A = [1 dt tau^2(-1*dt/tau+rm); 0 1 tau*(1-rm); 0 0 rm].
%   In a 2-D motion, the Jacobian is blkdiag(A, A)
%   In a 3-D motion, the Jacobian is blkdiag(A, A, A)
%   Positions are in meters; velocities are in meters/second; accelerations
%   are in meters/second^2.
%
%   Class support
%   -------------
%   state must be a finite double or single precision vector of the
%   dimensions specified above.
%   dt must be a real finite double or single precision scalar.
%   tau must be a real finite double or single precision vector or scalar.
%   If specified as vector, it must have the same number of elements as the
%   number of dimensions. If specified as a scalar, it will be expanded to
%   the number of dimensions.
%
%   Example
%   -------
%   % Define a state for 2-D Singer acceleration motion
%   state = [1;1;1;2;1;0]; 
%
%   % Calculate the Jacobian matrix assuming dt = 1 second
%   dfdx = SINGERJAC(state)
%
%   % Calculate the Jacobian matrix given dt = 0.1 second
%   dfdx = SINGERJAC(state, 0.1)
%
%   See also trackingEKF, initsingerekf, singer
   
%   References:
%   [1] Robert A. Singer, "Estimating Optimal Tracking Filter Performance
%       for Manned Maneuvering Targets". IEEE Transactions on Aerospace and
%       Electronic Systems Vol. AES-6, No. 4, July 1970.
%   [2] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
%       Tracking Systems", Artech House, 1999.
%   [3] X. Rong Li and Vasselin P. Jilkov, "A Survey of Maneuvering Target
%       Tracking: Dynamic Models", Proceedings of the SPIE Conference on
%       Signal and Data Processing of Small Targets, 2000.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

% Validate inputs
narginchk(1,3)

validateattributes(state, {'double','single'},...
    {'real', 'finite', 'vector', 'nonsparse'}, 'singerjac', 'state', 1);

classToUse = class(state);
numStates = numel(state);
cond = (numStates ~= 3) && (numStates ~= 6) && (numStates ~= 9);
coder.internal.errorIf(cond, 'shared_tracking:motion:incorrectStateVecWithInfo','[3 6 9]');

numDims = numStates / 3;

if nargin < 3
    tau = cast(20 * ones(numDims,1), classToUse);
else 
    validateattributes(varargin{2}, {'double','single'}, ...
        {'real', 'finite', 'positive', 'vector', 'nonsparse'}, ...
        'singerjac', 'tau', 3);
    if isscalar(varargin{2})
        tau = cast(varargin{2} * ones(numDims,1), classToUse);
    else
        validateattributes(varargin{2}, {'double','single'}, ...
            {'numel', numDims}, 'singerjac', 'tau', 3);
        tt = varargin{2};
        tau = cast(tt(:), classToUse);
    end
end

% If dt is not given, assume dt = 1
if nargin==1
    dt = ones(1, 1, classToUse);
else % dt is always the first optional argument
    validateattributes(varargin{1}, {'double','single'}, ...
        {'real', 'finite', 'scalar', 'nonsparse'}, 'singerjac', 'dt', 2);
    dt = cast(varargin{1}, classToUse);
end

dt2tau = dt./tau;
rm = exp(-dt2tau);
dfdx = zeros(numStates,numStates,classToUse);
for i = 1:numDims
    dfdx(3*i-2:3*i,3*i-2:3*i) = [1 dt (tau(i)^2)*(-1+dt2tau(i)+rm(i)); 0 1 tau(i)*(1-rm(i)); 0 0 rm(i)];
end