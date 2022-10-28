function scoreIncrement = trackScoreUpdate(wasDetected, varargin)
%trackScoreUpdate Calculates the increment to the track score
%   scoreIncrement = trackScoreUpdate(false, pd, pfa) calculates the 
%       (negative) score increment of a track that was not detected.
%   scoreIncrement = trackScoreUpdate(true, volume, likelihood, pd, pfa)
%       calculates the (positive) score increment of a track that was
%       detected.
%
% The function uses the following inputs:
%   wasDetected - A logical flag, true if the track was detected
%   volume      - Measurement volume element. 
%                 For example, the 4-D volume defined by a radar beamwidth
%                 in angle and the range and range rate bin widths.
%   likelihood  - Likelihood of the detection being assigned to the track.
%   pd          - The probability of detection.
%   pfa         - The probability of false alarm.
%
% Example:
% --------
% pd = 0.9;     % Probability of detection
% pfa = 1e-5;   % Probability of false alarm
% volume = 10;  % Volume of the sensor measurement bin
%
% %The initial track score is 5:
% score = 5;
%
% %Update the track score with a 'miss':
% scoreIncrement = fusion.internal.trackScoreUpdate(false, pd) % 'Miss'
% score = score + scoreIncrement
%
% %Update the track score with a 'hit':
% likelihood = 0.08; % likelihood of the detection assigned to the track
% scoreIncrement = fusion.internal.trackScoreUpdate(true, volume, likelihood, pd, pfa)
% score = score + scoreIncrement

% References:
% [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
% Tracking Systems", Artech House, 1999.

%   Copyright 2017 The MathWorks, Inc.

%#codegen

narginchk(2,6)
% If the track was not detected, the score increment (negative value) only
% depends on the probability of detection.
if ~wasDetected
    if nargin == 2
        pd = varargin{1};
        validateProb(pd, 'pd')
        scoreIncrement = log(1-pd); % Returns single if pd is single
    else
        pd = varargin{1};
        pfa = varargin{2};
        validateProb(pd, 'pd')
        validateProb(pfa, 'pfa')
        scoreIncrement = log((1-pd)/(1-pfa)); % Returns single if pd is single
    end
    return
end

% The other case is that the track was detected. If the track was detected,
% the score increment depends on the kinematics, as represented by the
% measurement likelihood, and on the signal itself.

% Parse inputs
[volume, likelihood, pd, pfa] = deal(varargin{:});

% Validate inputs:
validateProb(pd, 'pd')
validateProb(pfa, 'pfa')
validateattributes(volume, {'double','single'}, {'finite','real','scalar'}, mfilename, 'volume')
validateattributes(likelihood, {'double','single'}, {'finite', 'real', 'nonnegative', 'scalar'}, mfilename,'likelihood')
classToUse = class(pd);

% Cast everything to the class of pd
volumec = cast(volume,classToUse);
likelihoodc = cast(likelihood,classToUse);

dLk = log(volumec * likelihoodc); % ([1] 6.6 & 6.13)
dLs = log(pd/pfa); % ([1] 6.10b & 6.14)

scoreIncrement = dLk + dLs;
end

function validateProb(p, name)
validateattributes(p, {'double','single'}, ...
    {'finite','real','scalar','>=',0,'<=',1}, mfilename, name);
end
