function [posErrSq, velErrSq, accErrSq, posNEES, velNEES, accNEES] = errstateca(track, truth)
%ERRSTATECA State Estimation Errors for Constant Acceleration Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018 The MathWorks, Inc.

% track state is in form: [px; vx; ax; py; vy; ay; pz; vz; az]
trackPos = track.State([1 4 7]);
trackPosCov = track.StateCovariance([1 4 7],[1 4 7]);
trackVel = track.State([2 5 8]);
trackVelCov = track.StateCovariance([2 5 8],[2 5 8]);
trackAcc = track.State([3 6 9]);
trackAccCov = track.StateCovariance([3 6 9],[3 6 9]);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';
truthVel = truth.Velocity';
truthAcc = truth.Acceleration';

% compute estimation errors
posErr = trackPos - truthPos;
posErrSq = posErr' * posErr;
posNEES = posErr' / trackPosCov * posErr;

velErr = trackVel - truthVel;
velErrSq = velErr' * velErr;
velNEES = velErr' / trackVelCov * velErr;

accErr = trackAcc - truthAcc;
accErrSq = accErr' * accErr;
accNEES = accErr' / trackAccCov * accErr;
