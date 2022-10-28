function velNEES = neesvelcv(track, truth)
%NEESVELCV Velocity Normalized Estimation Error Squared for Constant Velocity Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; py; vy; pz; vz]
trackVel = track.State([2 4 6]);
trackVelCov = track.StateCovariance([2 4 6],[2 4 6]);

% truth position is in form: [px, py, pz]
truthVel = truth.Velocity';

% compute estimation errors
velErr = trackVel - truthVel;
velNEES = velErr' / trackVelCov * velErr;
