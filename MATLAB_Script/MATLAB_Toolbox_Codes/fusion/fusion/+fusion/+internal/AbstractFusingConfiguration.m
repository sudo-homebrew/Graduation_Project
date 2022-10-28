classdef (Abstract) AbstractFusingConfiguration < handle
% AbstractFusingConfiguration Interface definition for fuser source configuration
%  A fuser source configuration must satisfy the following public interface
%    Public properties:
%       SourceIndex
%       IsInternalSource
%       IsInitializingCentralTracks
%
%    Public methods:
%       centralTrack = transformToCentral(obj,localTrack);
%       localTrack   = transformToLocal(obj,centralTrack);
%       obj2 = clone(obj);
    
% Copyright 2019 The MathWorks, Inc.

%#codegen
    properties (Abstract)
        SourceIndex
        IsInternalSource
        IsInitializingCentralTracks
    end
    
    methods (Abstract)
        centralTrack = transformToCentral(obj,localTrack);
        localTrack   = transformToLocal(obj,centralTrack);
        obj2 = clone(obj);
    end
end