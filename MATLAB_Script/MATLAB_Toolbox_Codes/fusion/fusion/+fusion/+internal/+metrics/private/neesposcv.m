function posNEES = neesposcv(track, truth)
%NEESPOSCV Position Normalized Estimation Error Squared for Constant Velocity Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; py; vy; pz; vz]
trackPos = track.State([1 3 5]);
trackPosCov = track.StateCovariance([1 3 5],[1 3 5]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';

% compute estimation errors
posErr = trackPos - truthPos;
posNEES = posErr' / trackPosCov * posErr;
