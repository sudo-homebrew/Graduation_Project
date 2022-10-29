function resetImpl(obj)
%RESETIMPL Reset states of gnssSensor object

%   Copyright 2020 The MathWorks, Inc.

%#codegen

% Reset initial position estimate to the reference location.
obj.pInitPosECEF = fusion.internal.frames.lla2ecef(obj.ReferenceLocation);
% Reset initial velocity estimate to zero.
obj.pInitVelECEF = [0, 0, 0];

% Set the current time to the initial time of week in seconds.
obj.pCurrTime = obj.pTimeOfWeek;

% Reset random stream if needed.
if strcmp(obj.RandomStream, 'mt19937ar with seed')
    obj.pStream.reset;
end
end
