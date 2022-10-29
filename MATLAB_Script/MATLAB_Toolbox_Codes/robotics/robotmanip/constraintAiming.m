classdef constraintAiming < robotics.core.internal.mixin.SetProperties
    %constraintAiming Aiming constraint for pointing at a target location
    %   The AimingConstraint object describes a constraint that requires
    %   the z-axis of one body (the end effector) to aim at a target point
    %   on another body (the reference body). This constraint is satisfied
    %   if the aiming angle, a, between the z-axis of the end-effector
    %   frame and the line connecting the end-effector origin to the target
    %   point satisfies
    %
    %       abs(a) <= AngularTolerance
    %
    %   The position of the target point is defined relative to the
    %   reference body.
    %
    %   AC = constraintAiming(ENDEFFECTOR) returns an aiming
    %   constraint object, AC, that represents a constraint on a body whose
    %   name is given by ENDEFFECTOR.
    %
    %   AC = constraintAiming(ENDEFFECTOR, 'PropertyName',
    %   'PropertyValue') returns an aiming constraint object, AC, with each
    %   specified property set to the specified value
    %
    %   constraintAiming properties:
    %
    %   EndEffector       - Name of the end-effector body
    %   ReferenceBody     - Name of the reference body
    %   TargetPoint       - Position of the target relative to the reference body
    %   AngularTolerance  - Maximum allowed aiming angle (radians)
    %   Weights           - Weighting value for violations of this constraint
    %
    %
    %   Example:
    %
    %       % Create aiming constraint for a body named 'camera', relative
    %       % to the base frame
    %       aimCon = constraintAiming('camera');
    %
    %       % The z-axis of the body named 'camera' must point at the
    %       % target to within 10 degrees
    %       aimCon.AngularTolerance = deg2rad(10);
    %
    %       % Specify target point relative to the base frame
    %       aimCon.TargetPoint = [1, 0, 1];
    %
    %
    %   See also generalizedInverseKinematics,
    %   rigidBodyTree
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access=private)
        % Constant default property values
        ReferenceBodyDefault = ''
        TargetPointDefault = zeros(1,3)
        AngularToleranceDefault = 0
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
        ReferenceBody = constraintAiming.ReferenceBodyDefault
        
        %TargetPoint Position of the target relative to the reference body
        %
        %   Default: [0 0 0]
        TargetPoint = constraintAiming.TargetPointDefault

        %AngularTolerance Maximum allowed angle (in radians)
        %   This is an upper bound on the magnitude of the angle between
        %   the z-axis of the end-effector frame and the line connecting
        %   the end-effector origin to the target point.
        %
        %   Default: 0
        AngularTolerance = constraintAiming.AngularToleranceDefault
        
        %Weights Weighting value for violations of this constraint
        %
        %   Default: 1
        Weights = constraintAiming.WeightsDefault
        
    end
    
    properties (Access = protected)
        
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'TargetPoint', ...
            'AngularTolerance', 'Weights'};
                
        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintAiming.ReferenceBodyDefault, ...
                                            constraintAiming.TargetPointDefault, ...
                                            constraintAiming.AngularToleranceDefault, ...
                                            constraintAiming.WeightsDefault};
    end
    
    methods
        
        function obj = constraintAiming(endeffector, varargin)
            %constraintAiming Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end
        
        function newobj = copy(obj)
            %COPY Create copy of constraint with same property values
            %   NEWOBJ = COPY(OBJ) creates an constraintAiming object,
            %   NEWOBJ, with the same property values as OBJ. OBJ must be a
            %   scalar handle object.
            validateattributes(obj, {'constraintAiming'}, ...
                {'scalar'}, 'copy', 'obj');
            newobj = constraintAiming(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.TargetPoint = obj.TargetPoint;
            newobj.AngularTolerance = obj.AngularTolerance;
            newobj.Weights = obj.Weights;
        end
        
        function set.EndEffector(obj, value)
            obj.EndEffector = robotics.internal.validation.validateString(value, false, ...
                'constraintAiming', 'EndEffector');
        end
        
        function set.ReferenceBody(obj, value)
            obj.ReferenceBody = robotics.internal.validation.validateString(value, true, ...
                'constraintAiming', 'ReferenceBody');
        end
        
        function set.TargetPoint(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintAiming', 'TargetPoint', ...
                'numel', 3, 'nonnan', 'finite');
            obj.TargetPoint = double(reshape(value,1,3));
        end
        
        function set.AngularTolerance(obj, value)
            robotics.internal.validation.validateNumericMatrix(value, ...
                'constraintAiming', 'AngularTolerance', 'scalar', ...
                'nonnan', 'nonnegative');
            obj.AngularTolerance = double(value);
        end

        function set.Weights(obj, value)
            robotics.internal.validation.validateNumericMatrix( ...
                value, 'constraintAiming', 'Weights', ...
                'scalar', 'nonnan', 'finite', 'nonnegative');
            obj.Weights = double(value);
        end
        
    end
    
end
