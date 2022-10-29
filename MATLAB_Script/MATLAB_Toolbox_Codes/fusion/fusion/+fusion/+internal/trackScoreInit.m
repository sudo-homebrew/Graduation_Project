function initialScore = trackScoreInit(volume, beta, pd, pfa)
%trackScoreInit A function that initializes the score of a track
%   initialScore = trackScoreInit(pd, pfa, volume, beta) provides the track
%   initial score (log likelihood).
%   The initial score is calculated based on the following parameters:
%     beta      - Number of new targets in a unit volume (see below).
%     volume    - Measurement volume element. 
%                 For example, the 4-D volume defined by a radar beamwidth
%                 in angle and the range and range rate bin widths.
%     pd        - Probability of detection
%     pfa       - Probability of false alarm (false detection)
%
% Example:
% --------
% pd = 0.9;     % Probability of detection
% pfa = 1e-5;   % Probability of false alarm
% beta = 1e-2;  % New target density in a unit volume
% volume = 10;  % Volume of the sensor measurement bin
% score = trackScoreInit(volume, beta, pd, pfa)

% References:
% [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
% Tracking Systems", Artech House, 1999.

%   Copyright 2017 The MathWorks, Inc.

%#codegen

% Validate inputs:
narginchk(4,4);
validateProb(pd, 'pd')
validateProb(pfa, 'pfa')
validateattributes(volume, {'double','single'}, {'finite','real','scalar'}, mfilename, 'volume')
validateattributes(beta, {'double','single'}, {'finite','real','scalar'}, mfilename, 'beta')

% Cast everything to the type of pd
classToUse = class(pd);
pfac = cast(pfa,classToUse);
volumec = cast(volume,classToUse);
betac = cast(beta,classToUse);

% In the case of detection only data, the score is calculated using
betaFt = pfac/volumec;
initialScore = log(pd*betac/betaFt);
end

function validateProb(p, name)
validateattributes(p, {'double','single'}, ...
    {'finite','real','scalar','>=',0,'<=',1}, mfilename, name);
end