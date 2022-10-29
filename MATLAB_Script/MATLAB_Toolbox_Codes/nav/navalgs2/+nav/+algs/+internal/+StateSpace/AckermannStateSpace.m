classdef AckermannStateSpace < nav.StateSpace & nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%AckermannStateSpace An augmented SE2 state space that includes steering angle

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties (Access = ?nav.algs.internal.InternalAccess)
        MetricSpace
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess, Dependent)
        SkipStateValidation
    end
    
    methods
        function obj = AckermannStateSpace(ss, steerLimits)
        %AckermannStateSpaceHelper Augments an underlying metric space, such as SE2, with system-specific states
            if nargin == 1
                steerLimits = [-pi/4 pi/4];
            end
            bounds = [ss.StateBounds; steerLimits];
            
            obj = obj@nav.StateSpace('AckermannStateSpace',size(bounds,1),bounds);
            obj.MetricSpace = ss;
        end
        
        function d = distance(obj, s1, s2)
            d = obj.MetricSpace.distance(s1(:,1:3),s2(:,1:3));
        end
        
        function fullState = sampleUniform(obj, varargin)
            se2State = obj.MetricSpace.sampleUniform(varargin{:});
            fullState = [se2State zeros(size(se2State,1),1)];
        end
        
        function fullState = sampleGaussian(obj, varargin)
            se2State = obj.MetricSpace.sampleGaussian(varargin{:});
            fullState = [se2State zeros(size(se2State,1),1)];
        end
        
        function fullState = interpolate(obj, s1, s2, ratios)
            se2State = obj.MetricSpace.interpolate(s1(:,1:3), s2(:,1:3), ratios);
            d = s2(:,end)-s1(:,end);
            fullState = [se2State s1(:,end)+d.*ratios];
        end
        
        function stateOut = enforceStateBounds(obj, stateIn)
            stateOut = stateIn;
            stateOut(:,end) = min(max(obj.steerLimits(end,1),stateOut(:,end)),obj.steerLimits(end,2));
        end
        
        function cObj = copy(obj)
            cObj = nav.algs.internal.StateSpace.AckermannStateSpace(copy(obj.MetricSpace),obj.StateBounds(end,:));
        end
        
        function set.SkipStateValidation(obj, val)
            obj.MetricSpace.SkipStateValidation = val;
        end
        
        function val = get.SkipStateValidation(obj)
            val = obj.MetricSpace.SkipStateValidation;
        end
    end
    
    methods (Access = protected)
        function updateStateBounds(obj, bounds)
            obj.StateBoundsInternal = bounds;
            metricSz = obj.MetricSpace.NumStateVariables;
            obj.MetricSpace.StateBounds = obj.StateBoundsInternal(1:metricSz,:);
        end
    end
end
