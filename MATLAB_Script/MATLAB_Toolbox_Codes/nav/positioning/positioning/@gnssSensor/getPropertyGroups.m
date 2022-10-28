function groups = getPropertyGroups(obj)
%GETPROPERTYGROUPS Property group lists for gnssSensor

%   Copyright 2020 The MathWorks, Inc.

list.SampleRate = obj.SampleRate;
list.InitialTime = obj.InitialTime;
list.ReferenceLocation = obj.ReferenceLocation;
list.MaskAngle = obj.MaskAngle;
list.RangeAccuracy = obj.RangeAccuracy;
list.RangeRateAccuracy = obj.RangeRateAccuracy;
list.RandomStream = obj.RandomStream;
if ~isInactiveProperty(obj, 'Seed')
    list.Seed = obj.Seed;
end
groups = matlab.mixin.util.PropertyGroup(list);
end
