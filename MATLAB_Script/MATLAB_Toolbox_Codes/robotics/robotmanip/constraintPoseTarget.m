classdef constraintPoseTarget < robotics.core.internal.mixin.SetProperties
    %constraintPoseTarget Constraint on the relative pose of a body
    %   The constraintPoseTarget object describes a constraint on the pose of one
    %   body (the end effector) relative to another body (the reference
    %   body). This constraint is satisfied if the orientation of the end
    %   effector matches the target pose to within an angular tolerance
    %   specified by OrientationTolerance and the position of the end
    %   effector matches the target pose to within a positional tolerance
    %   specified by PositionTolerance.
    %
    %   PT = constraintPoseTarget(ENDEFFECTOR) returns an pose target
    %   object, PT, that represents a constraint on a body whose name is
    %   given by ENDEFFECTOR.
    %
    %   PT = constraintPoseTarget(ENDEFFECTOR, 'PropertyName',
    %   'PropertyValue') returns an pose target object, PT, with each
    %   specified property set to the specified value
    %
    %   constraintPoseTarget properties:
    %
    %   EndEffector          - Name of the end-effector body
    %   ReferenceBody        - Name of the reference body
    %   TargetTransform      - Desired pose relative to reference body
    %   OrientationTolerance - Maximum allowed rotation angle (radians)
    %   PositionTolerance    - Maximum allowed distance from target (meters)
    %   Weights              - Weighting values for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Create pose target constraint for a body named
    %       % 'tool' relative to the base frame
    %       pseTgt = constraintPoseTarget('tool');
    %
    %       % Allow deviations up to 0.01 m in position and 5 degrees in
    %       % orientation
    %       pseTgt.PositionTolerance = 0.01;
    %       pseTgt.OrientationTolerance = deg2rad(5);
    %
    %       % Set target pose
    %       pseTgt.TargetTransform = trvec2tform([1, 0, 1]);
    %
    %
    %   See also generalizedInverseKinematics,
    %   rigidBodyTree, constraintPoseTarget,
    %   constraintPositionTarget
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access=private)
        % Constant default property values
        ReferenceBodyDefault = ''
        TargetTransformDefault = eye(4)
        OrientationToleranceDefault = 0
        PositionToleranceDefault = 0
        WeightsDefault = [1,1]
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
        ReferenceBody = constraintPoseTarget.ReferenceBodyDefault
        
        %TargetTransform Desired pose relative to reference body
        %   This is a homogeneous transform specifying the target frame
        %   relative to the reference body. It is the transform that
        %   converts the expression of a point in the target frame to its
        %   expression in the reference body frame.
        %
        %   Default: eye(4)
        TargetTransform = constraintPoseTarget.TargetTransformDefault

        %OrientationTolerance Maximum allowed rotation angle (in radians)
        %   This is an upper bound on the magnitude of the rotation
        %   required to make the end-effector orientation match the target
        %   orientation.
        %
        %   Default: 0
        OrientationTolerance = constraintPoseTarget.OrientationToleranceDefault
        
        %PositionTolerance Maximum allowed distance from target (meters)
        %   This is an upper bound on the distance between the end-effector
        %   origin and the target position.
        %
        %   Default: 0
        PositionTolerance = constraintPoseTarget.PositionToleranceDefault
        
        %Weights Weighting values for violations of this constraint
        %   This is a two-element vector. The first element is a weight on
        %   violations of the orientation target and the second element is
        %   a weight on violations of the position target.
        %
        %   Default: [1 1]
        Weights = constraintPoseTarget.WeightsDefault
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'TargetTransform', ...
            'OrientationTolerance', 'PositionTolerance', 'Weights'};
        
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintPoseTarget.ReferenceBodyDefault, ...
                                            constraintPoseTarget.TargetTransformDefault, ...
                                            constraintPoseTarget.OrientationToleranceDefault, ...
                                            constraintPoseTarget.PositionToleranceDefault, ...
                                            constraintPoseTarget.WeightsDefault};
    end
    
    methods
        
        function obj = constraintPoseTarget(endeffector, varargin)
            %constraintPoseTarget Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates a constraintPoseTarget object, NEWOBJ,
            %   with the same property values as OBJ. OBJ must be a scalar
            %   handle object.
            validateattributes(obj, {'constraintPoseTarget'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintPoseTarget(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.TargetTransform = obj.TargetTransform;
            newobj.OrientationTolerance = obj.OrientationTolerance;
            newobj.PositionTolerance = obj.PositionTolerance;
            newobj.Weights = obj.Weights;
        end
        
        function set.EndEffector(obj, value)
            obj.EndEffector = robotics.internal.validation.validateString(value, false, ...
                'constraintPoseTarget', 'EndEffector');
        end
        
        function set.ReferenceBody(obj, value)
            obj.ReferenceBody = robotics.internal.validation.validateString(value, true, ...
                'constraintPoseTarget', 'ReferenceBody');
        end
        
        function set.TargetTransform(obj, value)
            robotics.internal.validation.validateHomogeneousTransform( ...
                value, 'constraintPoseTarget', 'TargetTransform', 'nonnan', 'finite');
            obj.TargetTransform = double(value);
        end
        
        function set.OrientationTolerance(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintPoseTarget', 'OrientationTolerance', 'scalar', ...
                'nonnan', 'nonnegative');
            obj.OrientationTolerance = double(value);
        end
        
        function set.PositionTolerance(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintPoseTarget', 'PositionTolerance', 'scalar', 'nonnan', ...
                'nonnegative');
            obj.PositionTolerance = double(value);
        end

        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintPoseTarget', 'Weights', ...
                'size', [1,2], 'nonnan', 'finite', 'nonnegative');
            obj.Weights = double(value);
        end
        
    end
    
end
