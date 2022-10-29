function [p, pdot] = pseudoranges(recPos, satPos, varargin)
%PSEUDORANGES Pseudoranges between GNSS receiver and satellites
%   p = PSEUDORANGES(recPos, satPos) returns the pseudoranges, in meters,
%   between the receiver at position recPos and the satellites at positions
%   satPos. The receiver position is specified in geodetic coordinates
%   (latitude-longitude-altitude) in (degrees, degrees, meters). The
%   satellite positions are specified as an S-by-3 matrix in meters in the
%   Earth-Centered-Earth-Fixed (ECEF) coordinate system. S is the number of
%   satellites.
%
%   [p, pdot] = PSEUDORANGES(___, recVel, satVel) returns the pseudorange
%   rates, pdot, in meters per second, between the receiver and satellites.
%   The receiver velocity, recVel, is specified in meters per second in the
%   North-East-Down (NED) coordinate system. The satellite velocities,
%   satVel, are specified as an S-by-3 matrix in meters per second in the
%   ECEF coordinate system. S is the number of satellites.
%
%   [p, pdot] = PSEUDORANGES(___, 'RangeAccuracy', rangeStd, ...
%   'RangeRateAccuracy', rangeRateStd) returns the pseudoranges and
%   pseudorange rates with random noises specified by rangeStd and
%   rangeRateStd, in meters and meters per second, respectively. The
%   default value of rangeStd and rangeRateStd are 1 and 0.02,
%   respectively.
%
%   Example:
%       recPos = [42 -71 50];
%       recVel = [1 2 3];
%       t = datetime('now');
%       [gpsSatPos, gpsSatVel] = gnssconstellation(t);
%       [p, pdot] = pseudoranges(recPos, gpsSatPos, recVel, gpsSatVel);
%
%   See also gnssconstellation, lookangles, receiverposition, gnssSensor.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

narginchk(2,8);

validateattributes(recPos, {'double', 'single'}, ...
    {'vector', 'numel', 3, 'real', 'finite'}, ...
    'pseudoranges', 'recPos', 1);
validateattributes(satPos, {'double', 'single'}, ...
    {'2d', 'ncols', 3, 'real', 'finite'}, ...
    'pseudoranges', 'satPos', 2);

% Parse optional inputs.
numOptArgs = numel(varargin);
validNumOptArgs = any(numOptArgs == [0 2 4 6]);
coder.internal.errorIf(~validNumOptArgs, 'MATLAB:minrhs');
if ~isempty(varargin) && isnumeric(varargin{1})
    recVel = varargin{1};
    satVel = varargin{2};
    validateattributes(recVel, {'double', 'single'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        'pseudoranges', 'recVel', 3);
    validateattributes(satVel, {'double', 'single'}, ...
        {'2d', 'nrows', size(satPos, 1), 'ncols', 3, 'real', 'finite'}, ...
        'pseudoranges', 'satVel', 4);
    optArgsStart = 3;
else
    recVel = zeros(size(recPos), 'like', recPos);
    satVel = zeros(size(satPos), 'like', satPos);
    optArgsStart = 1;
end
numOptArgs = numOptArgs - (optArgsStart-1);
defaults = struct('RangeAccuracy', 1, 'RangeRateAccuracy', 0.02);
props = matlabshared.fusionutils.internal.setProperties(defaults, ...
    numOptArgs, varargin{optArgsStart:end});
rangeStd = props.RangeAccuracy;
rangeRateStd = props.RangeRateAccuracy;

validateattributes(rangeStd, {'double', 'single'}, ...
    {'scalar', 'real', 'nonnegative'}, ...
    'pseudoranges', 'RangeAccuracy');
validateattributes(rangeRateStd, {'double', 'single'}, ...
    {'scalar', 'real', 'nonnegative'}, ...
    'pseudoranges', 'RangeRateAccuracy');

% Convert input receiver position and velocity.
recVel = fusion.internal.frames.ned2ecefv(recVel, recPos(:,1), recPos(:,2));
recPos = fusion.internal.frames.lla2ecef(recPos);

% Calculate pseudoranges and pseudorange rates using satellite and
% receiver positions and velocities.
[p, pdot] = nav.internal.gnss.calculatePseudoranges(satPos, satVel, ...
    recPos, recVel);

% Add measurement noises.
p = p + rangeStd .* randn(size(p), 'like', p);
pdot = pdot + rangeRateStd .* randn(size(pdot), 'like', pdot);
end
