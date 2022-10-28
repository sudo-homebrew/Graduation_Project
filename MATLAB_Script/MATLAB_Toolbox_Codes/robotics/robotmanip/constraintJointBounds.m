classdef constraintJointBounds < robotics.core.internal.mixin.SetProperties & ...
                      robotics.manip.internal.InternalAccess
    %constraintJointBounds Bounds on joint positions of a rigidBodyTree
    %   The constraintJointBounds object describes a constraint on the joint
    %   positions of a rigid body tree. This constraint is satisfied if
    %
    %       Bounds(:,1) <= q <= Bounds(:,2)
    %
    %   where q is a configuration vector for the rigid body tree.
    %
    %   JB = constraintJointBounds(RIGIDBODYTREE) returns a joint
    %   position bounds object, JB, that represents a constraint on
    %   configuration vectors of RIGIDBODYTREE.
    %
    %   JB = constraintJointBounds(RIGIDBODYTREE, 'PropertyName',
    %   'PropertyValue') returns a joint position bounds object, JB, with
    %   each specified property set to the specified value
    %
    %   constraintJointBounds properties:
    %
    %   Bounds  - Bounds on the configuration vector
    %   Weights - Weighting values for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Load example robots
    %       baxter = loadrobot('rethinkBaxter');
    %
    %       % Create joint position bound constraint for baxter
    %       jntBnd = constraintJointBounds(baxter);
    %
    %       % Restrict the joints to half of their usual range. That is, a
    %       % a joint whose limits in the rigid body tree are [-1, 2] is
    %       % now required to remain in the range [-0.25, 1.25].
    %       boundMean = mean(jntBnd.Bounds, 2);
    %       boundRange = diff(jntBnd.Bounds,1,2);
    %       jntBnd.Bounds(:,1) = boundMean - 0.5*0.5*boundRange;
    %       jntBnd.Bounds(:,2) = boundMean + 0.5*0.5*boundRange;
    %
    %
    %   See also generalizedInverseKinematics, rigidBodyTree

    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent)
        
        %Bounds Bounds on the configuration vector
        %   This is an N-by-2 array. Each row contains lower and upper
        %   limits on the corresponding element of the configuration
        %   vector.
        %
        %   Default: Joint limits specified by the rigid body tree model
        Bounds
        
        %Weights Weighting values for violations of this constraint
        %   This is an N-element vector. Each element is a weight on
        %   violating the bounds on the corresponding element of the
        %   configuration vector.
        %
        %   Default: ones(1,N)
        Weights
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'Bounds', 'Weights'};
        
                
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues
    end
    
    properties (Access = private)
        
        NumElements
        BoundsInternal
        WeightsInternal
        
    end
    
    methods
        
        function obj = constraintJointBounds(rigidbodytree, varargin)
            %constraintJointBounds Constructor
            
            %Convert optional inputs to strings. This is done with explicit
            %variable sizes to ensure code generation support
            numOptInputs = numel(varargin);
            optionInputs = cell(1,numOptInputs);
            for i = 1:numOptInputs
                optionInputs{i} = convertStringsToChars(varargin{i});
            end
            
            obj.BoundsInternal = constraintJointBounds.extractJointLimits(rigidbodytree);
            obj.NumElements = size(obj.BoundsInternal,1);
            obj.WeightsInternal = ones(1,obj.NumElements);
            obj.ConstructorPropertyDefaultValues = {obj.BoundsInternal, obj.WeightsInternal};
            obj.setProperties(numel(optionInputs), optionInputs{:});
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates a constraintJointBounds object,
            %   NEWOBJ, with the same property values as OBJ. OBJ must be a
            %   scalar handle object.
            validateattributes(obj, {'constraintJointBounds'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintJointBounds(obj.BoundsInternal);
            newobj.ConstructorPropertyDefaultValues = obj.ConstructorPropertyDefaultValues;
            newobj.Weights = obj.Weights;
        end
        
        function set.Bounds(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintJointBounds', 'Bounds', ...
                'size', [obj.NumElements,2], 'nonnan');
            obj.BoundsInternal = double(value);
        end
        
        function value = get.Bounds(obj)
            value = obj.BoundsInternal;
        end
        
        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintJointBounds', 'Weights', ...
                'size', [1,obj.NumElements], 'nonnan', 'finite', 'nonnegative');
            obj.WeightsInternal = double(value);
        end
        
        function value = get.Weights(obj)
            value = obj.WeightsInternal;
        end
        
    end
    
    methods (Static, Access = private)
        
        function limits = extractJointLimits(input)
            %extractJointLimits Get limits from matrix or rigid body tree
            %   LIMITS = extractJointLimits(INPUT) returns a N-by-2 array
            %   containing lower and upper bounds on each joint position.
            %   If INPUT is a rigidBodyTree object, LIMITS
            %   contains the joint limits defined by that object. If INPUT
            %   is a numeric array with two columns, LIMITS is equal to
            %   INPUT.
            if isnumeric(input)
                robotics.internal.validation.validateNumericMatrix(...
                    input, 'constraintJointBounds','rigidbodytree', ...
                    'ncols', 2);
                limits = double(input);
            else
                validateattributes(input, {'rigidBodyTree'}, ...
                    {'nonempty','scalar'},'constraintJointBounds','rigidbodytree');
                if input.NumNonFixedBodies == 0
                    robotics.manip.internal.error(...
                        'inversekinematics:RigidBodyTreeFixed');
                end
                limits = input.TreeInternal.JointPositionLimits;
            end
        end
        
    end
end
