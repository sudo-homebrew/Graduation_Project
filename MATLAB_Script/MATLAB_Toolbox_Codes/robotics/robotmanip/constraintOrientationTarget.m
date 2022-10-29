classdef constraintOrientationTarget < robotics.core.internal.mixin.SetProperties
    %constraintOrientationTarget Constraint on the relative orientation of a body
    %   The constraintOrientationTarget object describes a constraint that requires
    %   the orientation of one body (the end effector) to match a target
    %   orientation to within an angular tolerance in any direction. The
    %   target orientation is specified relative to the body frame of the
    %   reference body.
    %
    %   OT = constraintOrientationTarget(ENDEFFECTOR) returns an orientation
    %   target object, OT, that represents a constraint on a body whose
    %   name is given by ENDEFFECTOR.
    %
    %   OT = constraintOrientationTarget(ENDEFFECTOR, 'PropertyName',
    %   'PropertyValue') returns an orientation target object, OT, with
    %   each specified property set to the specified value
    %
    %   constraintOrientationTarget properties:
    %
    %   EndEffector          - Name of the end-effector body
    %   ReferenceBody        - Name of the reference body
    %   TargetOrientation    - Desired orientation relative to reference body
    %   OrientationTolerance - Maximum allowed rotation angle (radians)
    %   Weights              - Weighting value for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Create orientation target constraint for a body named
    %       % 'tool' relative to the base frame
    %       oriTgt = constraintOrientationTarget('tool');
    %
    %       % Allow deviations up to 5 degrees
    %       oriTgt.OrientationTolerance = deg2rad(5);
    %
    %       % Set target orientation
    %       oriTgt.TargetOrientation = eul2quat([pi/3, 0, 0]);
    %
    %
    %   See also generalizedInverseKinematics,
    %   rigidBodyTree
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen

    properties (Constant, Access=private)
        % Constant default property values
        ReferenceBodyDefault = ''
        TargetOrientationDefault = [1 0 0 0]
        OrientationToleranceDefault = 0
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
        ReferenceBody = constraintOrientationTarget.ReferenceBodyDefault
        
        %TargetOrientation Desired orientation relative to reference body
        %   This is a unit quaternion. The orientation of the end effector
        %   relative to the reference body frame is the orientation which
        %   converts a direction specified in the end effector frame to the
        %   same direction specified in the reference body frame.
        %
        %   Default: [1 0 0 0]
        TargetOrientation = constraintOrientationTarget.TargetOrientationDefault

        %OrientationTolerance Maximum allowed rotation angle (in radians)
        %   This is an upper bound on the magnitude of the rotation
        %   required to make the end-effector orientation match the target
        %   orientation
        %
        %   Default: 0
        OrientationTolerance = constraintOrientationTarget.OrientationToleranceDefault
        
        %Weights Weighting value for violations of this constraint
        %
        %   Default: 1
        Weights = constraintOrientationTarget.WeightsDefault
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'TargetOrientation', ...
            'OrientationTolerance', 'Weights'};
        
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintOrientationTarget.ReferenceBodyDefault, ...
                                            constraintOrientationTarget.TargetOrientationDefault, ...
                                            constraintOrientationTarget.OrientationToleranceDefault, ...
                                            constraintOrientationTarget.WeightsDefault};
    end
    
    methods
        
        function obj = constraintOrientationTarget(endeffector, varargin)
            %constraintOrientationTarget Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates an constraintOrientationTarget object,
            %   NEWOBJ, with the same property values as OBJ. OBJ must be a
            %   scalar handle object.
            validateattributes(obj, {'constraintOrientationTarget'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintOrientationTarget(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.TargetOrientation = obj.TargetOrientation;
            newobj.OrientationTolerance = obj.OrientationTolerance;
            newobj.Weights = obj.Weights;
        end
        
        function set.EndEffector(obj, value)
            obj.EndEffector = robotics.internal.validation.validateString(value, false, ...
                'constraintOrientationTarget', 'EndEffector');
        end
        
        function set.ReferenceBody(obj, value)
            obj.ReferenceBody = robotics.internal.validation.validateString(value, true, ...
                'constraintOrientationTarget', 'ReferenceBody');
        end
        
        function set.TargetOrientation(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintOrientationTarget', 'TargetOrientation', ...
                'numel', 4, 'nonnan', 'finite');
            validateattributes(sum(value(:).^2), {'numeric'}, {'nonzero'}, ...
                'constraintOrientationTarget', 'TargetOrientation');
            obj.TargetOrientation = double(robotics.internal.normalizeRows(reshape(value,1,4)));
        end
        
        function set.OrientationTolerance(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintOrientationTarget', 'OrientationTolerance', ...
                'scalar', 'nonnan', 'nonnegative');
            obj.OrientationTolerance = double(value);
        end

        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintOrientationTarget', 'Weights', ...
                'scalar', 'nonnan', 'finite', 'nonnegative');
            obj.Weights = double(value);
        end
        
    end
    
end
