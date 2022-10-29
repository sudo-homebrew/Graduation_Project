classdef dummyValidator < nav.StateValidator
%This class is for internal use only. It may be removed in the future.

%dummyValidator A helper class used in mobileRobotPropagator
%
%   The dummy validator is used when an environment object is not provided
%   to the mobileRobotPropagator object. Validation amounts to checking
%   whether states lie within the StateBounds.

%   Copyright 2021 The MathWorks, Inc.
    
    %#codegen
    
    properties
        StateLimits
        
        LowerLimits
        
        UpperLimits 
    end
    
    methods
        function obj = dummyValidator(stateSpace)
            obj = obj@nav.StateValidator(stateSpace);
            obj.StateLimits = stateSpace.StateBounds;
        end
        
        function isValid = isStateValid(obj, state)
        %isStateValid Checks whether passed states lie within the limits
        
            % Check if limits need to be updated
            if ~isequal(obj.StateSpace.StateBounds, obj.StateLimits)
                obj.StateLimits = obj.StateSpace.StateBounds;
            end
            
            % Verify that states fall within bounds
            n = size(state,1);
            isValid = all(state >= repmat(obj.LowerLimits,n,1) & ...
                state <= repmat(obj.UpperLimits,n,1),2);
        end
        
        function cObj = copy(obj)
            cObj = nav.algs.internal.dummyValidator(copy(obj.StateSpace));
        end
        
        function isValid = isMotionValid(~,~,~)
        %isMotionValid No-op
            isValid = true;
        end
        
        function set.StateLimits(obj, limits)
            obj.StateLimits = limits;
            updateLimits(obj,limits);
        end
        
        function updateLimits(obj, limits)
            obj.LowerLimits = limits(:,1)';
            obj.UpperLimits = limits(:,2)';
        end
    end
end
