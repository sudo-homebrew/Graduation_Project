function Q = singerProcessNoise(state, varargin)
%singerProcessNoise  Process noise matrix for Singer accleration model
%   Q = singerProcessNoise(STATE) returns an N-by-N Singer acceleration
%   process noise matrix, where N is the number of elements in the state
%   vector, STATE. It uses the following default values:
%     T      - Time step, default: 1 second.
%     tau    - Target maneuver time constant, default: 20 seconds.
%     sigmam - Target maneuver standard deviation: 1 m/sec^2
%
%   Q = singerProcessNoise(STATE, T), additionally, allows you to specify
%   the time step, T.
%
%   Q = singerProcessNoise(STATE, T, TAU), additionally, allows you to
%   specify the target maneuver time constant, TAU.
%
%   Q = singerProcessNoise(STATE, T, TAU, SIGMAM), additionally, allows you
%   to sepcify the target maneuver standar deviation, SIGMAM.
%
%   Class support
%   -------------
%   STATE must be a finite real double or single precision vector with 3,
%   6, or 9 elements, corresponding to [pos;vel;acc] in 1, 2, or 3
%   dimensions, respectively. 
%   T must be a real finite double or single precision scalar.
%   TAU must be a real finite positive double or single precision vector or
%   scalar. If specified as vector, it must have the same number of
%   elements as the number of dimensions. If specified as a scalar, it will
%   be expanded to the number of dimensions.
%   SIGMAM must be a real finite positive double or single precision vector
%   or scalar. If specified as vector, it must have the same number of
%   elements as the number of dimensions. If specified as a scalar, it will
%   be expanded to the number of dimensions.
%
%   Example
%   -------
%   % Get the Singer process noise for a 3-D Singer state with default time
%   % step, target maneuver time constant, and standard deviation
%   Q1 = singerProcessNoise((1:9)')
%
%   % Get the Singer Process noise for a 3-D Singer state with time step of
%   % 2 seconds, target maneuver time constant of 10 seconds in x-axis and
%   % y-axis and 100 seconds in z-axis, and target maneuver standard
%   % deviation of 1 in x- and y- axes and 0 in z-axis.
%   Q2 = singerProcessNoise((1:9)', 2, [10 10 100], [1 1 0])
%
%   See also: singer, singerjac, initsingerekf

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


narginchk(1,4)
% Validate state
validateattributes(state, {'double','single'},...
    {'real', 'finite', 'vector', 'nonsparse'}, 'singerProcessNoise', 'state', 1);

numStates = numel(state);
cond = (numStates ~= 3) && (numStates ~= 6) && (numStates ~= 9);
coder.internal.errorIf(cond, 'shared_tracking:motion:incorrectStateVecWithInfo','[3 6 9]');

classToUse = class(state);
numDims = numStates / 3;

% If dt is not given, assume dt = 1
if nargin==1
    dt = ones(1, 1, classToUse);
else % dt is always the first optional argument
    validateattributes(varargin{1}, {'double','single'}, ...
        {'real', 'finite', 'scalar', 'nonsparse'}, 'singerProcessNoise', 'dt', 2);
    dt = cast(varargin{1}, classToUse);
end

if nargin < 3
    tau = cast(20 * ones(numDims,1), classToUse);
else
    validateattributes(varargin{2}, {'double','single'}, ...
        {'real', 'finite', 'positive', 'vector', 'nonsparse'}, ...
        'singerProcessNoise', 'tau', 2);
    if isscalar(varargin{2})
        tau = cast(varargin{2} * ones(numDims,1), classToUse);
    else
        validateattributes(varargin{2}, {'double','single'}, ...
            {'numel', numDims}, 'singerProcessNoise', 'tau', 3);
        tt = varargin{2};
        tau = cast(tt(:), classToUse);
    end
end

% If sigmam is not given, assume sigmam = 1
if nargin < 4
    sigmam = ones(numDims, 1, classToUse);
else % sigmam is always the third optional argument
    validateattributes(varargin{3}, {'double','single'}, ...
        {'real', 'finite', 'nonnegative', 'nonsparse', 'vector'}, 'singerProcessNoise', 'sigmam', 4);
    if isscalar(varargin{3})
        sigmam = cast(varargin{3} * ones(numDims,1), classToUse);
    else
        validateattributes(varargin{3}, {'double','single'}, ...
            {'numel', numDims}, 'singerProcessNoise', 'tau', 4);
        sg = varargin{3};
        sigmam = cast(sg(:), classToUse);
    end
end

dt2tau = dt./tau;
rowm = exp(-dt2tau);

% Expand to 3-D
q11 = 2./tau.*sigmam.^2 .* (tau.^5)./2 .* (1 - rowm.^2 + 2.*dt2tau + 2.*dt2tau.^3./3 - 2.*dt2tau.^2 - 4.*dt2tau.*rowm);
q12 = 2./tau.*sigmam.^2 .* (tau.^4)./2 .* (rowm.^2 + 1 - 2.*rowm + 2.*dt2tau.*rowm - 2.*dt2tau + dt2tau.^2);
q13 = 2./tau.*sigmam.^2 .* (tau.^3)./2 .* (1 - rowm.^2 - 2.*dt2tau.*rowm);
q22 = 2./tau.*sigmam.^2 .* (tau.^3)./2 .* (4.*rowm - 3 - rowm.^2 + 2.*dt2tau);
q23 = 2./tau.*sigmam.^2 .* (tau.^2)./2 .* (rowm.^2 + 1 - 2.*rowm);
q33 = 2./tau.*sigmam.^2 .* (tau./2) .* (1 - rowm.^2);

Q = zeros(numStates, numStates, classToUse);
for i = 1:numDims
    Q(3*i-2:3*i, 3*i-2:3*i) = [q11(i) q12(i) q13(i); q12(i) q22(i) q23(i); q13(i) q23(i) q33(i)];
end