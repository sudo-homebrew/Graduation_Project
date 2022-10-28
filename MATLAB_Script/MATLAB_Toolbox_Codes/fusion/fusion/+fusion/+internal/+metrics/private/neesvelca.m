function velNEES = neesvelca(track, truth)
%NEESVELCA Velocity Normalized Estimation Error Squared for Constant Acceleration Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; ax; py; vy; ay; pz; vz; az]
trackVel = track.State([2 5 8]);
trackVelCov = track.StateCovariance([2 5 8],[2 5 8]);

% truth position is in form: [px, py, pz]
truthVel = truth.Velocity';

% compute estimation errors
velErr = trackVel - truthVel;
velNEES = velErr' / trackVelCov * velErr;
