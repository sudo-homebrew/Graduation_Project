function V = ssmussvTolerances(userMuOpt, gUBmax, dynRange, Focus, peakOnly)
% Need to clean up, but fine for now.  July 31, 2015.  Sets up most
% tolerances for the calculation.  

%   Copyright 1986-2020 The MathWorks, Inc.
V.userMuOpt = userMuOpt;
V.KLB = 0;    % unchanged since 2013
V.KUB = inf;  % unchanged since 2013
V.dynRange = dynRange;
V.pTolInit = 0.5;
V.osborneCondNumber = 1e5; % 1e10;  % No noticeable diff with 1e5
V.gUBmax = gUBmax;

% ETOL is used in LMICarve, to dictate how wide an interval can be carved,
% with the "constraint" that the upper-bound over the interval is within
% etol of the point bound at the "center".  There are other factors at
% play, but/so this convergence to 3*ETOL is guaranteed.
V.etol = 0.002;
% if peakOnly
%    V.etol = 0.002;
% else
%    V.etol = 0.02;
% end

% Tolerance used for absolute value on MU.  This is used in LMICarve,
% ssmussvPeak, and ssmussvCurve. The upper bound is set to V.abstol
% rather than letting it go to zero.
V.abstol = 0.0001; 

% The PPD value controls the "tolerance" per-se, as to how narrow the
% intervals can get and be considered "done", without having achieved a
% specified convergence (in bound) tolerance.
PPD = 1000; % -> RelIntervalTol = 1.0012
V.RelIntervalTol = exp(log(10)/PPD);
V.AbsIntervalTol = 1e-4*max(dynRange(1),Focus(1));


% Peak vs. Curve
V.peakOnly = peakOnly;
V.PeakFraction = 0.333;  % for curve

% Default behavior is to use slower algorithm that provides bounds on a
% denser frequency grid.  Use 'P' option in userMuOpt for fast algorithm.

% if peakOnly
%    % Use "fast" algorithm that gives bounds on a coarse grid
%    V.gUBmax = gUBmax;
%    V.TightFactor = 1;
% else
%    % Use "slower" algorithm that gives more informative bounds on a dense grid
%    V.gUBmax = gUBmax;
%    V.TightFactor = 0.02;
% end
