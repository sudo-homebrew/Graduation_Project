classdef Assignment < handle
%

%   Copyright 2018 The MathWorks, Inc.

    methods (Abstract)
       tm = metricsTable(obj, deleted)     
       tm = buildSummary(obj, deleted)
       copyIndexedProperties(obj, iProps, src, iSrc)
       iAppended = appendIndexedProperties(obj, src, iSrc)
       computeCumulativeMetrics(obj)
       setIdentifier(obj, id)
       id = getIdentifier(obj)
    end
end
