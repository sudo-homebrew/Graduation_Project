function absVelErr = abserrvelca(track, truth)
%ABSERRVELCA Absolute Velocity Estimation Error for Constant Acceleration Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [px; vx; ax; py; vy; ay; pz; vz; az]
trackVel = track.State([2 5 8]);

% truth position is in form: [px, py, pz]
truthVel = truth.Velocity';

% compute estimation errors
velErr = trackVel - truthVel;
absVelErr = sqrt(velErr' * velErr);
