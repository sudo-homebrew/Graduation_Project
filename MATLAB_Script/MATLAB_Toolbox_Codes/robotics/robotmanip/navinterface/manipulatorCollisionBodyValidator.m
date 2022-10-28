classdef manipulatorCollisionBodyValidator < nav.StateValidator
%MANIPULATORCOLLISIONBODYVALIDATOR Validate a state of a rigid body tree against a cell-array of convex collision bodies
%   The validator performs a state and motion validity check of a
%   state that corresponds to the joint configuration of a rigid body tree.
%
%   SV = manipulatorCollisionBodyValidator creates a state validator with a default
%   manipulatorStateSpace object with default values for validation distance and
%   environment.
%
%   SV = manipulatorCollisionBodyValidator(SS) creates a state
%   validator based on the given manipulator state space object, SS. The
%   environment of the manipulator is empty.
%
%   SV = manipulatorCollisionBodyValidator(_,"PropertyName",PropertyValue)
%   sets properties on the object using name-value arguments.
%
%   MANIPULATORCOLLISIONBODYVALIDATOR Properties:
%       StateSpace          - State space used for validation
%       ValidationDistance  - Distance resolution for validating motion between configurations
%       IgnoreSelfCollision - Ignore self-collision checks
%       Environment         - Obstacles in the environment 
%
%   MANIPULATORCOLLISIONBODYVALIDATOR  Methods:
%       copy            - Create deep copy of state validator
%       isStateValid    - Validate the input joint state
%       isMotionValid   - Validate the motion between two states
%
%   Example:
%   sv = manipulatorCollisionBodyValidator;
%   
%   % Check for a valid state.
%   isStateValid(sv, sampleUniform(sv.StateSpace))
%
%   % Add a collisionBox to the environment.
%   sv.Environment{end+1} = collisionBox(0.1,0.1,0.1);

%   Copyright 2021 The MathWorks, Inc.

%#codegen
    properties(Access = private)
        StateValidatorInternal
    end

    properties (Access = {?nav.algs.internal.InternalAccess})
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation = false
    end

    properties
        %ValidationDistance - Distance resolution for validating motion between configurations
        %   The validation distance determines the number of interpolated nodes
        %   between two adjacent nodes of the tree that should be checked for
        %   validity.
        %
        %   Default: 0.1
        ValidationDistance = 0.1

        %IgnoreSelfCollision - Ignore self-collision checks
        %   To ignore collisions between manipulator bodies while 
        %   validating states or motion, set this property to true or 1.
        %
        %   Default: false
        IgnoreSelfCollision = false

        %Environment - Obstacles in the environment 
        %   Obstacles in the environment, specified as a cell array of 
        %   collision objects like collisionMesh or collisionBox.
        %
        %   Default: {}
        Environment
    end

    methods
        function set.Environment(obj, env)
            obj.validateEnvironment(env);
            tempenv = env;
            for i = 1:length(tempenv)
                tempenv{i} = copy(tempenv{i});
            end
            obj.Environment = tempenv;
            obj.updateValidator();
        end

        function set.ValidationDistance(obj, valdist)
            robotics.internal.validation.validatePositiveNumericScalar(valdist,...
                                                              getClassName(obj),...
                                                              obj.ValidationDistancePropertyName);
            obj.ValidationDistance = valdist;
            obj.updateValidator();
        end

        function set.IgnoreSelfCollision(obj, flag)
            obj.IgnoreSelfCollision = robotics.internal.validation.validateLogical(flag,...
                getClassName(obj), ...
                obj.IgnoreSelfCollisionPropertyName);
            obj.updateValidator();
        end

    end

    properties(Constant, Access=private)
        %EnvironmentPropertyName
        EnvironmentPropertyName = 'Environment';

        %ValidationDistancePropertyName
        ValidationDistancePropertyName = 'ValidationDistance';

        %IgnoreSelfCollisionPropertyName
        IgnoreSelfCollisionPropertyName = 'IgnoreSelfCollision';

    end

    properties(Constant, Access=private)
        %DefaultValidationDistance
        DefaultValidationDistance = 0.1

        %DefaultEnvironment
        DefaultEnvironment = {}

        %DefaultIgnoreSelfCollision
        DefaultIgnoreSelfCollision = false
    end

    methods(Access=private, Static)
        function parser = makeParser()
        %Construct a name-value pair parser for the properties
            names = {
                manipulatorCollisionBodyValidator.EnvironmentPropertyName,...
                manipulatorCollisionBodyValidator.ValidationDistancePropertyName, ...
                manipulatorCollisionBodyValidator.IgnoreSelfCollisionPropertyName ...
                    };
            defaults = {
                manipulatorCollisionBodyValidator.DefaultEnvironment,...
                manipulatorCollisionBodyValidator.DefaultValidationDistance ...
                manipulatorCollisionBodyValidator.DefaultIgnoreSelfCollision, ...
                       };
            parser = robotics.core.internal.NameValueParser(names, defaults);
        end
    end

    methods(Access=private)
        function validateEnvironment(obj, env)
        %validateEnvironment Validation of input env for checkCollision
        %   The env are passed as a cell-array of collision
        %   objects. The function returns true if the env are a
        %   valid input to checkCollision
            validateattributes(env, {'cell'}, {}, ...
                               getClassName(obj),...
                               obj.EnvironmentPropertyName);
            for i = 1:length(env)
                if(~isa(env{i}, 'robotics.core.internal.CollisionGeometryBase'))
                    robotics.manip.internal.error(...
                        'manipulatorplanning:InvalidCollisionBodyEnvironment');
                end
            end
        end

        function name = getClassName(~)
            name = 'manipulatorCollisionBodyValidator';
        end

        function updateValidator(obj)
            obj.StateValidatorInternal.IgnoreSelfCollision = obj.IgnoreSelfCollision;
            obj.StateValidatorInternal.Environment = obj.Environment;
            obj.StateValidatorInternal.ValidationDistance = obj.ValidationDistance;
        end
    end

    methods
        function obj = manipulatorCollisionBodyValidator(varargin)
            parser = manipulatorCollisionBodyValidator.makeParser();
            if(nargin == 0)
                ss = manipulatorStateSpace;
                parser.parse(varargin{:});
            else
                ss = varargin{1};
                parser.parse(varargin{2:end});
            end
            obj@nav.StateValidator(ss);
            env = ...
                parser.parameterValue(obj.EnvironmentPropertyName);
            ignoreSelfCollision = ...
                parser.parameterValue(obj.IgnoreSelfCollisionPropertyName);
            validationDistance = ...
                parser.parameterValue(obj.ValidationDistancePropertyName);
            obj.StateValidatorInternal = robotics.manip.internal.ManipulatorStateValidator(...
                ss, env,validationDistance);
            obj.Environment = env;
            obj.ValidationDistance = validationDistance;
            obj.IgnoreSelfCollision = ignoreSelfCollision;
        end

        function newObj = copy(obj)
        %copy Create a deep copy of the state validator
            newObj = manipulatorCollisionBodyValidator(...
                copy(obj.StateSpace), ...
                'Environment', obj.Environment, ...
                'ValidationDistance', obj.ValidationDistance);
        end

        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
        %isMotionValid Validate the motion between two states
        %   [ISVALID, LASTVALID] = isMotionValid(SV, STATE1, STATE2)
        %   interpolates between STATE1 and STATE2 validates each state in the
        %   interpolation. The function also returns the last valid state
        %   along the interpolated path.
            
            narginchk(3, 3);
            if(~obj.SkipStateValidation)
                nav.internal.validation.validateStateVector(...
                    state1, obj.StateSpace.NumStateVariables, ...
                    'isMotionValid', 'state1');
                nav.internal.validation.validateStateVector(...
                    state2, obj.StateSpace.NumStateVariables, ...
                    'isMotionValid', 'state2');
            end
            [isValid, lastValid] = obj.StateValidatorInternal.isMotionValid(state1, state2);
        end

        function isValid = isStateValid(obj, state)
        %isStateValid Validate the input joint state
        %   ISVALID = isStateValid(SV, STATE) checks if the input state of the
        %   rigid body tree robot model is in collision with the environment or
        %   itself, or is out of state bounds. STATE must be an
        %   N-by-NumStateVariables matrix, representing N state samples. ISVALID
        %   is an N-element vector of logical values, with TRUE indicating that
        %   the corresponding state sample in STATE is valid.
            
            narginchk(2, 2);
            if(~obj.SkipStateValidation)
                nav.internal.validation.validateStateMatrix(...
                    state, nan, obj.StateSpace.NumStateVariables, ...
                    'isStateValid', 'state');
            end
            numStates = size(state, 1);
            lowerBound = repmat(obj.StateSpace.StateBounds(:, 1)', numStates, 1);
            upperBound = repmat(obj.StateSpace.StateBounds(:, 2)', numStates, 1);
            isInBounds = all((lowerBound <= state) & (state <= upperBound), 2);
            isValid = isInBounds & obj.StateValidatorInternal.isStateValid(state);
        end

    end
end
