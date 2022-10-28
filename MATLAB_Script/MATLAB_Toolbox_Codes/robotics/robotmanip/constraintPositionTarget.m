classdef constraintPositionTarget < robotics.core.internal.mixin.SetProperties
    %constraintPositionTarget Constraint on the relative position of a body
    %   The constraintPositionTarget object describes a constraint on the position of
    %   one body (the end effector) relative to the body frame of another
    %   body (the reference body). This constraint is satisfied if the
    %   position of the end-effector origin matches the target position to
    %   within a specified tolerance. The target position is defined
    %   relative to the specified reference body.
    %
    %   PT = constraintPositionTarget(ENDEFFECTOR) returns a position target
    %   object, PT, that represents a constraint on a body whose name is
    %   given by ENDEFFECTOR.
    %
    %   PT = constraintPositionTarget(ENDEFFECTOR, 'PropertyName',
    %   'PropertyValue') returns a position target object, PT, with each
    %   specified property set to the specified value
    %
    %   constraintPositionTarget properties:
    %
    %   EndEffector       - Name of the end-effector body
    %   ReferenceBody     - Name of the reference body
    %   TargetPosition    - Desired position relative to reference body
    %   PositionTolerance - Maximum allowed distance from target (meters)
    %   Weights           - Weighting value for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Create position target constraint for a body named
    %       % 'tool' relative to the base frame
    %       posTgt = constraintPositionTarget('tool');
    %
    %       % Allow deviations up to 0.01 m
    %       posTgt.PositionTolerance = 0.01;
    %
    %       % Set target position
    %       posTgt.TargetPosition = [1, 0, 1];
    %
    %
    %   See also generalizedInverseKinematics,
    %   rigidBodyTree
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access=private)
        % Constant default property values
        ReferenceBodyDefault = ''
        TargetPositionDefault = zeros(1,3)
        PositionToleranceDefault = 0
        WeightsDefault = 1
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
        ReferenceBody = constraintPositionTarget.ReferenceBodyDefault
        
        %TargetPosition Desired position relative to reference body
        %
        %   Default: [0 0 0]
        TargetPosition = constraintPositionTarget.TargetPositionDefault
        
        %PositionTolerance Maximum allowed distance from target (meters)
        %   This is an upper bound on the distance between the end-effector
        %   origin and the target position.
        %
        %   Default: 0
        PositionTolerance = constraintPositionTarget.PositionToleranceDefault
        
        %Weights Weighting value for violations of this constraint
        %
        %   Default: 1
        Weights = constraintPositionTarget.WeightsDefault
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'TargetPosition', ...
            'PositionTolerance', 'Weights'};
        
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintPositionTarget.ReferenceBodyDefault, ...
                                            constraintPositionTarget.TargetPositionDefault, ...
                                            constraintPositionTarget.PositionToleranceDefault, ...
                                            constraintPositionTarget.WeightsDefault};
        
    end
    
    methods
        
        function obj = constraintPositionTarget(endeffector, varargin)
            %constraintPositionTarget Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates a constraintPositionTarget object, NEWOBJ,
            %   with the same property values as OBJ. OBJ must be a scalar
            %   handle object.
            validateattributes(obj, {'constraintPositionTarget'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintPositionTarget(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.TargetPosition = obj.TargetPosition;
            newobj.PositionTolerance = obj.PositionTolerance;
            newobj.Weights = obj.Weights;
        end
        
        function set.EndEffector(obj, value)
            obj.EndEffector = robotics.internal.validation.validateString(value, false, ...
                'constraintPositionTarget', 'EndEffector');
        end
        
        function set.ReferenceBody(obj, value)
            obj.ReferenceBody = robotics.internal.validation.validateString(value, true, ...
                'constraintPositionTarget', 'ReferenceBody');
        end
        
        function set.TargetPosition(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintPositionTarget', 'TargetPosition', ...
                'numel', 3, 'nonnan', 'finite');
            obj.TargetPosition = double(reshape(value,1,3));
        end
        
        function set.PositionTolerance(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintPositionTarget', 'PositionTolerance', ...
                'scalar', 'nonnan', 'nonnegative');
            obj.PositionTolerance = double(value);
        end
        
        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintPositionTarget', 'Weights', ...
                'scalar', 'nonnan', 'finite', 'nonnegative');
            obj.Weights = double(value);
        end
        
    end
    
end
