function [posErrSq, velErrSq, yawRateErrSq, posNEES, velNEES, yawRateNEES] = errstatect(track, truth)
%ERRSTATECT State Estimation Errors for Constant Turn Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018 The MathWorks, Inc.

% track state is in form: [x; vx; y; vy; omega; z; vz]
trackPos = track.State([1 3 6]);
trackPosCov = track.StateCovariance([1 3 6],[1 3 6]);
trackVel = track.State([2 4 7]);
trackVelCov = track.StateCovariance([2 4 7],[2 4 7]);
trackYawRate = track.State(5);
trackYawRateCov = track.StateCovariance(5,5);

% truth position is in form: [px, py, pz]
truthPos = truth.Position';
truthVel = truth.Velocity';
truthYawRate = truth.AngularVelocity(3);

% compute estimation errors
posErr = trackPos - truthPos;
posErrSq = posErr' * posErr;
posNEES = posErr' / trackPosCov * posErr;

velErr = trackVel - truthVel;
velErrSq = velErr' * velErr;
velNEES = velErr' / trackVelCov * velErr;

yawRateErr = trackYawRate - truthYawRate;
yawRateErrSq = yawRateErr' * yawRateErr;
yawRateNEES = yawRateErr' / trackYawRateCov * yawRateErr;

