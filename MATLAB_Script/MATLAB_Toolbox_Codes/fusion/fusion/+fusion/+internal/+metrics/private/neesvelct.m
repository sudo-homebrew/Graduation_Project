function velNEES = neesvelct(track, truth)
%NEESVELCT Velocity Normalized Estimation Error Squared for Constant Turn Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [x; vx; y; vy; omega; z; vz]
trackVel = track.State([2 4 7]);
trackVelCov = track.StateCovariance([2 4 7],[2 4 7]);

% truth position is in form: [px, py, pz]
truthVel = truth.Velocity';

% compute estimation errors
velErr = trackVel - truthVel;
velNEES = velErr' / trackVelCov * velErr;
