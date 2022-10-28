function absPosErr = abserrposca(track, truth)
%ABSERRPOSCA Absolute Position Estimation Error for Constant Acceleration Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; ax; py; vy; ay; pz; vz; az]
trackPos = track.State([1 4 7]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';

% compute estimation errors
posErr = trackPos - truthPos;
absPosErr = sqrt(posErr' * posErr);
