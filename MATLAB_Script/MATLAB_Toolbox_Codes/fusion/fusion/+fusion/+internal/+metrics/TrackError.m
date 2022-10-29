classdef TrackError < fusion.internal.metrics.Error
%

%   Copyright 2018 The MathWorks, Inc.

    methods
        function obj = TrackError
            obj.IdentifierFcn = @defaultTrackIdentifier;
            obj.InvalidID = NaN;
            obj.IDLabel = 'TrackID';
        end
    end
end
