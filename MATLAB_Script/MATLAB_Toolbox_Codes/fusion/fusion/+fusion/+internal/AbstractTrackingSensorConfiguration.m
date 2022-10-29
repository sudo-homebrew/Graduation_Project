classdef (Abstract) AbstractTrackingSensorConfiguration < handle
    % This is an internal class and may be modified or removed in a future
    % release. 
    
    % Copyright 2019-2020 The MathWorks, Inc.
    
    %#codegen
    properties (Abstract)
        IsValidTime
        SensorIndex
        ClutterDensity
        MaxNumDetsPerObject
    end
    
    methods
        Pd = probDetection(obj,states,varargin);
        filter = initialize(obj,detections);
        N = expectedNumDets(obj,stateSamples);
        sync(obj,minObj);
        obj2 = clone(obj);
    end
end