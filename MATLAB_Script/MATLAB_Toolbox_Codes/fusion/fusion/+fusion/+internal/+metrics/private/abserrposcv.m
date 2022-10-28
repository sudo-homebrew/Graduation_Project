function absPosErr = abserrposcv(track, truth)
%ABSERRPOSCV Absolute Position Estimation Error for Constant Velocity Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; py; vy; pz; vz]
trackPos = track.State([1 3 5]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';

% compute estimation errors
posErr = trackPos - truthPos;
absPosErr = sqrt(posErr' * posErr);

