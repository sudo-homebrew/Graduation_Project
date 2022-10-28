classdef (Abstract) StateValidator < handle
%STATEVALIDATOR Create state validator for path planning
%   StateValidator is an interface for all state validators used for path planning.
%   Derive from this class if you are defining your own state validator.
%   This representation allows state and motion validation.
%
%   To create a template for deriving from the nav.StateSpace class, use the
%   createPlanningTemplate function.
%
%   VALIDATOR = nav.StateValidator(SPACE) creates a state validator object
%   VALIDATOR that validates states in the state space SPACE.
%   This constructor can only be called from a derived class.
%
%   StateValidator properties:
%      StateSpace        - State space used for validation
%
%   StateValidator methods:
%      copy              - Create deep copy of object
%      isStateValid      - Check if state is valid
%      isMotionValid     - Check if path between states is valid
%
%   See also validatorOccupancyMap, createPlanningTemplate.

%   Copyright 2019 The MathWorks, Inc.

    %#codegen
    properties (SetAccess = immutable)
        %StateSpace - State space used for validation
        %   The state inputs to isStateValid and isMotionValid need to be
        %   samples from this state space.
        StateSpace
    end

    methods
        function obj = StateValidator(stateSpace)
        %STATEVALIDATOR Constructor for StateValidator object

            narginchk(1,1);

            nav.internal.validation.validateStateSpace(stateSpace, ...
                                                       'StateValidator', 'stateSpace');
            obj.StateSpace = stateSpace;
        end
    end

    methods (Abstract)

        %isStateValid Check if state is valid
        isValid = isStateValid(obj, state)

        %isMotionValid Check if path between states is valid
        [isValid, lastValidState] = isMotionValid(obj, state1, state2)

        %COPY Create deep copy of object
        copyObj = copy(obj)

    end
end
