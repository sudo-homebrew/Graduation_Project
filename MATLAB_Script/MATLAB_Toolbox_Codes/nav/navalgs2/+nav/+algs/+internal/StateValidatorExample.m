%   This class defines a template for creating a custom state validation definition
%   that can be used by the sampling-based path planners like plannerRRT and
%   plannerRRTStar. The state validator allows you to validate states and
%   motions between states.
%
%   To access documentation for how to define a state space, enter the following
%   at the MATLAB command prompt:
%
%    >> doc nav.StateValidator
%
%   For a concrete implementation of the same interface, see the following
%   nav.StateValidator class:
%
%    >> edit validatorOccupancyMap
%
%
%   To use this custom state validator for path planning, follow the steps
%   outlined below and complete the class definition. Then, save this
%   file somewhere on the MATLAB path. You can add a folder to the
%   path using the ADDPATH function.

%   Copyright 2019 The MathWorks, Inc.


classdef StateValidatorExample < nav.StateValidator & ...
        matlabshared.planning.internal.EnforceScalarHandle
    
    %---------------------------------------------------------------------
    % Step 1: Define properties to be used by your state space. These are
    % user-defined properties.
    properties
        
        %------------------------------------------------------------------
        % Place your code here
        %------------------------------------------------------------------
        
    end
    
    %----------------------------------------------------------------------
    % Step 2: Define functions used for managing your state validator.
    methods
        % a) Use the constructor to set the name of the state space, the
        %    number of state variables, and to define its boundaries.
        %
        function obj = StateValidatorExample(space)
            
            narginchk(0,1)
            
            if nargin == 0
                space = stateSpaceSE2;
            end
            
            % The state space object is validated in the StateValidator base class
            obj@nav.StateValidator(space);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % b) Define how the object is being copied (a new object is created
        %    from the current one). You have to customize this function
        %    if you define your own properties or special constructor.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/copy
        %
        function copyObj = copy(obj)
            
            % Default behavior: Create a new object of the same type with no arguments.
            copyObj = feval(class(obj), obj.StateSpace);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % c) Define how a given state is validated. The STATE input can be a
        %    single state (row) or multiple states (one state per row).
        %    You have to customize this function if you want to have
        %    special validation behavior, for example to check for
        %    collisions with obstacles.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isStateValid
        %
        function isValid = isStateValid(obj, state)
            
            narginchk(2,2);
            
            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                "isStateValid", "state");
            
            % Default behavior: Verify that state is within bounds
            bounds = obj.StateSpace.StateBounds';
            inBounds = state >= bounds(1,:) & state <= bounds(2,:);
            isValid = all(inBounds, 2);
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
        % d) Define how a motion between states is validated.
        %
        %    For more help, see
        %    >> doc nav.StateValidator/isMotionValid
        %
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            
            narginchk(3,3);
            
            state1 = nav.internal.validation.validateStateVector(state1, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state1");
            state2 = nav.internal.validation.validateStateVector(state2, ...
                obj.StateSpace.NumStateVariables, "isMotionValid", "state2");
            
            if (~obj.isStateValid(state1))
                error("statevalidator:StartStateInvalid", "The start state of the motion is invalid.");
            end
            
            % Default behavior: Interpolate with some fixed interval
            % between state1 and state2 and validate the interpolated
            % points.
            numInterpPoints = 100;
            interpStates = obj.StateSpace.interpolate(state1, state2, linspace(0,1,numInterpPoints));
            interpValid = obj.isStateValid(interpStates);
            firstInvalidIdx = find(~interpValid, 1);
            
            if isempty(firstInvalidIdx)
                isValid = true;
                lastValid = state2;
            else
                isValid = false;
                lastValid = interpStates(firstInvalidIdx-1,:);
            end
            
            %--------------------------------------------------------------
            % Place your code here
            %--------------------------------------------------------------
        end
        
    end
end

