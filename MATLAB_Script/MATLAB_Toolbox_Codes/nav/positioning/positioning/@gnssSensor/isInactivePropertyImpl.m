function tf = isInactivePropertyImpl(obj, prop)
%ISINACTIVEPROPERTYIMPL Return true if input property is inactive

%   Copyright 2020 The MathWorks, Inc.

tf = false;
if strcmp(prop, 'Seed')
    if strcmp(obj.RandomStream, 'Global stream')
        tf = true;
    end
end
end