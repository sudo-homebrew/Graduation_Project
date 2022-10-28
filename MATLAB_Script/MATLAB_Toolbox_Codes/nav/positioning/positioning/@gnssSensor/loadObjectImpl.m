function loadObjectImpl(obj, s, wasLocked)
%LOADOBJECTIMPL Load gnssSensor object

%   Copyright 2020 The MathWorks, Inc.

% Load public properties.
loadObjectImpl@matlab.System(obj, s, wasLocked);

% Load private values of dependent properties.
obj.pTimeZone = s.pTimeZone;
obj.pFormat = s.pFormat;
obj.pGPSWeek = s.pGPSWeek;
obj.pTimeOfWeek = s.pTimeOfWeek;
    
% Load private properties.
if wasLocked
    obj.pInputPrototype = s.pInputPrototype;
    obj.pInitPosECEF = s.pInitPosECEF;
    obj.pInitVelECEF = s.pInitVelECEF;
    obj.pCurrTime = s.pCurrTime;
    obj.pRefFrame = s.pRefFrame;
    
    if strcmp(s.RandomStream, 'mt19937ar with seed')
        obj.pStream = RandStream('mt19937ar', ...
            'seed', obj.Seed);
        if ~isempty(s.pStreamState)
            obj.pStream.State = s.pStreamState;
        end
    end
end
end