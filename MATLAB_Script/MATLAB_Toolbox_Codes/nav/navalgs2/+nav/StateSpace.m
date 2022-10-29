classdef (Abstract) StateSpace < handle
%STATESPACE Create state space for path planning
%   StateSpace is an interface for all state spaces used for path planning.
%   Derive from this class if you are defining your own state space.
%   This representation allows sampling, interpolation, and calculating
%   the distance between states.
%
%   To create a template for deriving from the nav.StateSpace class, use the
%   createPlanningTemplate function.
%
%   SS = nav.StateSpace("NAME", NUMSTATEVARS, BOUNDS) creates a state space object SS
%   with a given name, NAME, and the state bounds, BOUNDS. The number
%   of state variables is given by NUMSTATEVARS.
%   This constructor can only be called from a derived class.
%
%   StateSpace properties:
%      Name              - Name of state space
%      NumStateVariables - Number of state variables in space
%      StateBounds       - Bounds of state variables
%
%   StateSpace methods:
%      copy               - Create deep copy of the state space object
%      distance           - Distance between two states.
%      enforceStateBounds - Ensure state lies within state bounds
%      interpolate        - Interpolate between two states
%      sampleUniform      - Sample state using a uniform distribution
%      sampleGaussian     - Sample state using a Gaussian distribution
%
%   See also stateSpaceSE2, createPlanningTemplate.

%   Copyright 2019-2021 The MathWorks, Inc.

    %#codegen
    properties (SetAccess = protected)
        %Name - Name of the state space
        Name
    end

    properties (SetAccess = immutable)
        %NumStateVariables - Dimension of the state
        NumStateVariables
    end

    properties (Dependent)
        %StateBounds - Bounds of state variables
        %   The property contains one row for each state variable.
        %   Each row is a 1-by-2 vector of [min,max].
        StateBounds
    end

    properties (Access = protected)
        %StateBoundsInternal - Internal storage for dependent StateBounds
        StateBoundsInternal
    end

    methods
        function obj = StateSpace(name, numStateVars, bounds)
        %STATESPACE Constructor for StateSpace object

            narginchk(3,3);

            % Validate state space name
            nameChar = robotics.internal.validation.validateString(convertStringsToChars(name), ...
                                                              false, 'StateSpace', 'name');
            obj.Name = nameChar;

            % Validate number of state variables
            validateattributes(numStateVars, {'double'}, {'nonempty', 'scalar', 'integer'}, ...
                               'StateSpace', 'numStateVariables'); 
            obj.NumStateVariables = numStateVars;

            % Validate state bounds
            validBounds = obj.validateStateBounds(bounds, numStateVars, 'StateSpace', 'bounds');

            obj.StateBoundsInternal = validBounds;
        end
    end

    methods (Abstract)

        %DISTANCE Distance between two states
        dist = distance(obj, state1, state2)

        %INTERPOLATE Interpolate between two states
        state = interpolate(obj, state1, state2, ratios)

        %sampleGaussian Sample state using Gaussian distribution
        state = sampleGaussian(obj, meanState, stdDev)

        %sampleUniform Sample state using uniform distribution
        state = sampleUniform(obj, varargin)

        %enforceStateBounds Ensure that state lies within state bounds
        boundedState = enforceStateBounds(obj, state)

        %COPY Create deep copy of state space object
        copyObj = copy(obj)

    end

    methods (Access = protected)
        function updateStateBounds(obj, ~)
            %updateStateBounds
            % No-op
        end
    end

    methods

        function set.StateBounds(obj, bounds)
        %set.StateBounds Setter for StateBounds dependent property

            obj.StateBoundsInternal = obj.validateStateBounds(bounds, ...
                                                              obj.NumStateVariables, 'StateSpace', 'StateBounds');
            obj.updateStateBounds(bounds);
        end

        function bounds = get.StateBounds(obj)
        %get.StateBounds Getter for StateBounds dependent property

            bounds = obj.StateBoundsInternal;
        end

    end

    methods (Static, Access = protected)
        function validateNumSamples(numSamples, fcnName, varName)
        %validateNumSamples Validate input for number of samples

            validateattributes(numSamples, {'double'}, {'nonempty', 'scalar', 'real', ...
                                'nonnan', 'finite', 'integer', '>=', 1}, fcnName, varName);
        end
    end

    % Input validation for several user-facing methods. These
    % validation functions are "protected", so they can only be called by 
    % custom state spaces derived from nav.StateSpace, or those created 
    % by createPlanningTemplate.
    methods (Access = {?nav.StateSpace, ?nav.algs.internal.InternalAccess})
        function [state1Valid, state2Valid, distValid] = ...
                validateInterpolateInput(obj, state1, state2, ratios)
            %validateInterpolateInput Validate inputs to interpolate function

            % Make sure that state vectors are rows for consistency.
            state1Valid = nav.internal.validation.validateStateVector(state1, obj.NumStateVariables, 'interpolate', 'state1');
            state2Valid = nav.internal.validation.validateStateVector(state2, obj.NumStateVariables, 'interpolate', 'state2');
            
            validateattributes(ratios, {'double'}, {'nonempty', 'vector', '>=', 0, '<=' 1}, 'interpolate', 'ratios');
            distValid = ratios(:)';
        end

        function [numSamples, stateBounds, sampleNear] = validateSampleUniformInput(obj, varargin)
        %validateSampleUniformInput Validate inputs to sampleUniform function

            sampleNear = false;
            stateBounds = obj.StateBounds;
            if nargin == 1
                % Syntax: sampleUniform(OBJ)
                numSamples = 1;
            elseif nargin == 2
                % Syntax: sampleUniform(OBJ, NUMSAMPLES)
                obj.validateNumSamples(varargin{1}, 'sampleUniform', 'numSamples');
                if varargin{1} > 1
                    numSamples = varargin{1};
                else
                    numSamples = 1;
                end
            elseif nargin >= 3
                % Syntax: sampleUniform(OBJ, NEARSTATE, DIST)
                
                % Need to sample near a given state
                sampleNear = true;
                
                % Validate nearState/dist and convert to row-vectors
                
                nearState = nav.internal.validation.validateStateVector(varargin{1}, ...
                    obj.NumStateVariables, 'sampleUniform', 'nearState');
                validateattributes(varargin{2}, {'double'}, {'nonempty', 'vector', ...
                    'numel', obj.NumStateVariables, 'positive'}, 'sampleUniform', 'dist');
                dist = varargin{2}(:)';
                % Sample based on distance to limits
                distBounds = [nearState - dist; nearState + dist];
                
                % Transpose the output to correspond with
                % StateBounds format.
                stateBounds = obj.enforceStateBounds(distBounds)';
                
                if nargin == 4
                    % Syntax: sampleUniform(OBJ, NEARSTATE, DIST, NUMSAMPLES)
                    obj.validateNumSamples(varargin{3}, 'sampleUniform', 'numSamples');
                    numSamples = varargin{3};
                else
                    numSamples = 1;
                end
            else
                numSamples = 1;
                narginchk(1,4)
            end
        end

        function [meanState, stdDev, numSamples] = validateSampleGaussianInput(obj, meanState, stdDev, varargin)
        %validateSampleGaussianInput Validate inputs to sampleGaussian function

            meanState = nav.internal.validation.validateStateVector(meanState, ...
                obj.NumStateVariables, 'sampleGaussian', 'meanState');
            stdDev = nav.internal.validation.validateStateVector(stdDev, ...
                obj.NumStateVariables, 'sampleGaussian', 'stdDev');
            
            if nargin > 3
                obj.validateNumSamples(varargin{1}, 'sampleGaussian', 'numSamples');
                numSamples = varargin{1};
            else
                numSamples = 1;
            end
            
            % Ensure that our stdDev vector forms the diagonal of a positive 
            % definite matrix
            if any(stdDev <= 0)
                validateattributes(stdDev, {'double'}, {'positive'}, 'sampleGaussian', 'stdDev')
            end
        end
    end
    
    methods (Static, Access = {?nav.StateSpace, ?nav.algs.internal.InternalAccess})
        function [expandedState1, expandedState2] = validateDistanceInput(numStateVars, state1, state2)
            %validateDistanceInput Validate inputs to distance function
            %   There are 3 different valid input sizes parsed:
            %   (1) STATE1 is a matrix, STATE2 is a matrix with the same
            %     number of rows as STATE1. The output equals the input.            
            %   (2) STATE1 is a vector, STATE2 is a matrix. In EXPANDEDSTATE1, 
            %     STATE1 will be expanded into a matrix with the same size as STATE2.
            %   (3) STATE1 is a matrix, STATE2 is a vector. In EXPANDEDSTATE2, 
            %     STATE2 will be expanded into a matrix with the same size as STATE1.
            
            % Get number of states for each input
            s1 = size(state1, 1);
            s2 = size(state2, 1);
            
            nav.internal.validation.validateStateMatrix(state1, nan, numStateVars, 'distance', 'state1');
            
            if s1 ~= s2
                % If inputs do not have same number of states, ensure that
                % one of them only contains a single state                                
                nav.internal.validation.validateStateMatrix(state2, nan, numStateVars, 'distance', 'state2');
                
                % Find difference between states
                if s1 == 1
                    expandedState1 = repelem(state1, s2, 1);
                    expandedState2 = state2;
                elseif s2 == 1
                    expandedState1 = state1;
                    expandedState2 = repelem(state2, s1, 1);
                else
                    % Assign values for codegen
                    expandedState1 = state1;
                    expandedState2 = state2;
                    
                    % Throw error for incorrect size
                    nav.internal.validation.validateStateMatrix(state1, size(state2,1), numStateVars, 'distance', 'state1');
                end
            else % otherwise both states have to have the same number of rows
                
                nav.internal.validation.validateStateMatrix(state2, size(state1,1), numStateVars, 'distance', 'state2');
                
                % Keep states as-is
                expandedState1 = state1;
                expandedState2 = state2;
            end

        end
    end

    methods (Static, Access = ?nav.algs.internal.InternalAccess)
        function validBounds = validateStateBounds(bounds, numStateVars, funcName, varName)
        %validateStateBounds Input validation for state bounds

            validateattributes(bounds, {'double'}, ...
                               {'nonempty', 'real', 'nonnan', 'size', [numStateVars 2]}, ...
                               funcName, varName);

            % Confirm that lower bounds are smaller or equal to upper bounds
            if any(bounds(:,1) > bounds(:,2))
                coder.internal.error("nav:navalgs:statespace:UpperBoundTooSmall");
            end

            validBounds = double(bounds);
        end
    end
    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
        % Let the coder know about non-tunable parameters
            props = {'NumStateVariables'};
        end
    end
    
end
