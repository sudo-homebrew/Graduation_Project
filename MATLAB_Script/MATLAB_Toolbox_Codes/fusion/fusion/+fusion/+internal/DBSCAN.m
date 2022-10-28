classdef DBSCAN < phased.internal.AbstractClusterDBSCAN
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020-2021, The MathWorks, Inc.
    
    % This class exposes a light-weight call to DBSCAN algorithm by
    % trackerGridRFS.
    
    %#codegen
    methods
        function obj = DBSCAN(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods
        function idx = cluster(obj,x)
            idx = clusterdataDBSCAN(obj,x);
        end
    end
    
    methods (Static)
        function clusterIdx = clusterUsingDistance(distance,epsilon,minNumPts)
            clusterIdx = zeros(size(distance,1),1,'uint32');
            [~,~,clusterIdx(:)] = fusion.internal.DBSCAN.computeClusters(distance,epsilon,minNumPts);
        end
    end
end