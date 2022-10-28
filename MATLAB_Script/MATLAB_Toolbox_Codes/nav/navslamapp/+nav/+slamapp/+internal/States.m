classdef States < int32
%This class is for internal use only. It may be removed in the future.

%STATES Enumeration of SLAM app states

%   Copyright 2018 The MathWorks, Inc.

    enumeration
        Init (0)
        LoadingBag (1)
        LoadingFromWS (2)
        SensorDataReady (3)
        Inspecting (4)
        Mapping (5)
        MappingPaused (6)
        ModifyingLoopClosure (7)
        ModifyingIncremental (8)
        Mapped (9)
    end
end
