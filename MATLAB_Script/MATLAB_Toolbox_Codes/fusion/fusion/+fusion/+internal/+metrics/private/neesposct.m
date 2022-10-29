function posNEES = neesposct(track, truth)
%NEESPOSCT Position Normalized Estimation Error Squared for Constant Turn Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [x; vx; y; vy; omega; z; vz]
trackPos = track.State([1 3 6]);
trackPosCov = track.StateCovariance([1 3 6],[1 3 6]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';

% compute estimation errors
posErr = trackPos - truthPos;
posNEES = posErr' / trackPosCov * posErr;
