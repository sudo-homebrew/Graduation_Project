function [posErrSq, velErrSq, posNEES, velNEES] = errstatecv(track, truth)
%ERRSTATECV State Estimation Errors for Constant Velocity Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018 The MathWorks, Inc.

% track state is in form: [px; vx; py; vy; pz; vz]
trackPos = track.State([1 3 5]);
trackVel = track.State([2 4 6]);
trackPosCov = track.StateCovariance([1 3 5],[1 3 5]);
trackVelCov = track.StateCovariance([2 4 6],[2 4 6]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';
truthVel = truth.Velocity';

% compute estimation errors
posErr = trackPos - truthPos;
posErrSq = posErr' * posErr;
posNEES = posErr' / trackPosCov * posErr;

velErr = trackVel - truthVel;
velErrSq = velErr' * velErr;
velNEES = velErr' / trackVelCov * velErr;
