classdef TruthError < fusion.internal.metrics.Error
%

%   Copyright 2018 The MathWorks, Inc.

    methods
        function obj = TruthError
            obj.IdentifierFcn = @defaultTruthIdentifier;
            obj.InvalidID = NaN;
            obj.IDLabel = 'TruthID';
        end
    end
end
