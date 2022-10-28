classdef constraintCartesianBounds < robotics.core.internal.mixin.SetProperties
    %constraintCartesianBounds Constraint to keep a body origin inside Cartesian bounds
    %   The constraintCartesianBounds object describes a constraint on the position
    %   of one body (the end effector) relative to a target frame fixed on
    %   another body (the reference body). This constraint is satisfied if
    %   the position , p, of the end-effector origin relative to the target
    %   frame satisfies
    %
    %       Bounds(i,1) <= p(i) <= Bounds(i,2) for i = 1, 2, 3.
    %
    %   The target frame is defined by the TargetTransform, which is the
    %   homogeneous transformation that converts points in the Target frame
    %   to points in the ReferenceBody frame.
    %
    %   CB = constraintCartesianBounds(ENDEFFECTOR) returns an Cartesian
    %   bounds object, CB, that represents a constraint on a body whose
    %   name is given by ENDEFFECTOR.
    %
    %   CB = constraintCartesianBounds(ENDEFFECTOR, 'PropertyName',
    %   'PropertyValue') returns an Cartesian bounds object, CB, with each
    %   specified property set to the specified value
    %
    %   constraintCartesianBounds properties:
    %
    %   EndEffector     - Name of the end-effector body
    %   ReferenceBody   - Name of the reference body
    %   TargetTransform - Pose of the target relative to the reference body
    %   Bounds          - Bounds on end-effector position relative to target
    %   Weights         - Weighting values for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Create Cartesian Bounds constraint for a body named
    %       % 'tool' relative to the base frame
    %       crtBnd = constraintCartesianBounds('tool');
    %
    %       % Set bounds on the position of the end-effector origin
    %       % relative to the target frame. In this case, the x and y
    %       % coordinates must smaller than 0.01 m, but the z coordinate is
    %       % unconstrained
    %       crtBnd.Bounds = [-0.01, 0.01; -0.01, 0.01; -Inf, Inf];
    %
    %       % Specify target-frame pose
    %       crtBnd.TargetTransform = trvec2tform([1, 0, 1]);
    %
    %
    %   See also generalizedInverseKinematics,
    %   rigidBodyTree
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen

    properties (Constant, Access=private)
        % Constant default property values
        ReferenceBodyDefault = ''
        TargetTransformDefault = eye(4)
        BoundsDefault = zeros(3,2)
        WeightsDefault = ones(1,3)
    end
        
    properties
        
        %EndEffector Name of the end-effector body
        %   When this constraint is passed to a
        %   generalizedInverseKinematics solver, EndEffector must
        %   be the name of a body in the solver's rigid body tree.
        EndEffector
        
        %ReferenceBody Name of the reference body
        %   When this constraint is passed to a
        %   generalizedInverseKinematics solver, ReferenceBody
        %   must be either the name of a body in the solver's rigid body
        %   tree or an empty character vector. An empty character vector
        %   indicates that the constraint is specified relative to the base
        %   of the rigid body tree.
        %
        %   Default: ''
        ReferenceBody = constraintCartesianBounds.ReferenceBodyDefault
        
        %TargetTransform Pose of the target relative to the reference body
        %   This is a homogeneous transform specifying the target frame
        %   relative to the reference body. It is the transform that
        %   converts the expression of a point in the target frame to its
        %   expression in the reference body frame.
        %
        %   Default: eye(4)
        TargetTransform = constraintCartesianBounds.TargetTransformDefault
        
        %Bounds Bounds on end-effector position expressed in the target frame
        %   This is a 3 x 2 array. Each row contains lower and upper limits
        %   on the corresponding element of the relative position.
        %
        %   Default: zeros(3,2)
        Bounds = constraintCartesianBounds.BoundsDefault
        
        %Weights Weighting values for violations of this constraint
        %   This is a three-element vector. Each element is a weight on
        %   violating the bounds on the corresponding element of the
        %   relative position.
        %
        %   Default: [1 1 1]
        Weights = constraintCartesianBounds.WeightsDefault
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'TargetTransform', ...
            'Bounds', 'Weights'};
                
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintCartesianBounds.ReferenceBodyDefault, ...
                                            constraintCartesianBounds.TargetTransformDefault, ...
                                            constraintCartesianBounds.BoundsDefault,...
                                            constraintCartesianBounds.WeightsDefault};
    end
    
    methods
        
        function obj = constraintCartesianBounds(endeffector, varargin)
            %constraintCartesianBounds Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates a constraintCartesianBounds object,
            %   NEWOBJ, with the same property values as OBJ. OBJ must be a
            %   scalar handle object.
            validateattributes(obj, {'constraintCartesianBounds'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintCartesianBounds(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.TargetTransform = obj.TargetTransform;
            newobj.Bounds = obj.Bounds;
            newobj.Weights = obj.Weights;
        end
        
        function set.EndEffector(obj, value)
            obj.EndEffector = robotics.internal.validation.validateString(value, false, ...
                'constraintCartesianBounds', 'EndEffector');
        end
        
        function set.ReferenceBody(obj, value)
            obj.ReferenceBody = robotics.internal.validation.validateString(value, true, ...
                'constraintCartesianBounds', 'ReferenceBody');
        end
        
        function set.TargetTransform(obj, value)
            robotics.internal.validation.validateHomogeneousTransform( ...
                value, 'constraintCartesianBounds', 'TargetTransform', 'nonnan', 'finite');
            obj.TargetTransform = double(value);
        end
        
        function set.Bounds(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintCartesianBounds', 'Bounds', ...
                'size', [3,2], 'nonnan');
            obj.Bounds = double(value);
        end
        
        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintCartesianBounds', 'Weights', ...
                'size', [1,3], 'nonnan', 'finite', 'nonnegative');
            obj.Weights = double(value);
        end
        
    end
    
end
