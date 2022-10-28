function setupImpl(obj, pos, ~)
%SETUPIMPL Setup gnssSensor object

%   Copyright 2020 The MathWorks, Inc.

%#codegen

% Store input for type casting.
obj.pInputPrototype = pos;

% Store reference frame.
obj.pRefFrame = fusion.internal.frames.ReferenceFrame.getMathObject( ...
    obj.ReferenceFrame);
            
% Setup Random Stream object if required.
if strcmp(obj.RandomStream, 'mt19937ar with seed')
    if isempty(coder.target)
        obj.pStream = RandStream('mt19937ar', 'seed', obj.Seed);
    else
        obj.pStream = coder.internal.RandStream('mt19937ar', 'seed', ...
            obj.Seed);
    end
end
end
