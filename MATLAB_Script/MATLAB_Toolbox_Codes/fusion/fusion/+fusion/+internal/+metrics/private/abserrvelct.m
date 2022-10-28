function absVelErr = abserrvelct(track, truth)
%ABSERRVELCT Absolute Velocity Estimation Error for Constant Turn Model
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

% track state is in form: [x; vx; y; vy; omega; z; vz]
trackVel = track.State([2 4 7]);

% truth position is in form: [px, py, pz]
truthVel = truth.Velocity';

% compute estimation errors
velErr = trackVel - truthVel;
absVelErr = sqrt(velErr' * velErr);
