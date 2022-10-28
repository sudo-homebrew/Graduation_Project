classdef constraintRevoluteJoint < robotics.core.internal.mixin.SetProperties
%constraintRevoluteJoint Revolute joint constraint between two bodies.
%   constraintRevoluteJoint defines a revolute joint constraint between a
%   successor and a predecessor body on the rigidBodyTree. The properties
%   PredecessorTransform and SuccessorTransform define intermediate frames
%   with respect to the predecessor and successor bodies, respectively. The
%   constraint allows rotation along the Z-axes of the intermediate frames
%   when the constraint is satisfied. The constraint is satisfied when the
%   successor body's intermediate frame's origin coincides with the
%   predecessor body's intermediate frame's origin and the Z-axes of the
%   intermediate frames align.
%
%   CR = constraintRevoluteJoint(SUCCESSORBODY,PREDECESSORBODY) returns a
%   revolute joint constraint object, CR, that represents a constraint
%   between the body named SUCCESSORBODY and the body named
%   PREDECESSORBODY.
%
%   CR = constraintRevoluteJoint(_,Name=Value) returns a revolute joint
%   constraint CR, with each specified property set to the specified value.
%
%   constraintRevoluteJoint properties:
%
%   SuccessorBody        - Name of joint's successor body
%   PredecessorBody      - Name of joint's predecessor body
%   PredecessorTransform - Transform of predecessor intermediate frame
%   SuccessorTransform   - Transform of successor intermediate frame
%   PositionTolerance    - Position tolerance of joint constraint (in
%                          meters)
%   OrientationTolerance - Orientation tolerance of joint constraint (in
%                          radians)
%   JointPositionLimits  - Limits on constraint's joint position (in
%                          radians)
%   Weights              - Weights on value for violations of this
%                          constraint
%
%   Example:
%
%       % Create a revolute joint constraint between
%       % successor body "body1" and predecessor body "body2"
%       constrRevolute = constraintRevoluteJoint("body1","body2");
%
%       % Define the intermediate frame on the predecessor body
%       constrRevolute.PredecessorTransform = trvec2tform([1,0,0]);
%
%   See also constraintFixedJoint, constraintPrismaticJoint,
%   generalizedInverseKinematics, rigidBodyTree

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Constant, Access=private)
        PredecessorTransformDefault = eye(4);
        SuccessorTransformDefault = eye(4);
        PositionToleranceDefault = 0
        JointPositionLimitsDefault = [-pi, pi];
        OrientationToleranceDefault = 0
        WeightsDefault = [1,1,1]
    end

    properties
        %SuccessorBody Name of the joint constraint's successor body
        %   When this constraint is passed to a generalizedInverseKinematics
        %   solver, SuccessorBody must be the name of a body in the solver's
        %   rigid body tree.
        SuccessorBody

        %PredecessorBody Name of the joint constraint's predecessor body
        %   When this constraint is passed to a generalizedInverseKinematics
        %   solver, PredecessorBody must be the name of a body in the solver's
        %   rigid body tree.
        PredecessorBody

        %PredecessorTransform Fixed transform of predecessor intermediate frame
        %   This is the fixed transform of the intermediate frame on the
        %   predecessor body defined with respect to the predecessor body.
        %
        %   Default: eye(4)
        PredecessorTransform = constraintRevoluteJoint.PredecessorTransformDefault

        %SuccessorTransform Fixed transform of successor intermediate frame
        %   This is the fixed transform of the intermediate frame on the
        %   successor body defined with respect to the successor body.
        %
        %   Default: eye(4)
        SuccessorTransform = constraintRevoluteJoint.SuccessorTransformDefault

        %PositionTolerance Position tolerance of joint constraint (in meters).
        %   Tolerance on the distance between the origins of the
        %   predecessor and successor intermediate frames defined by the
        %   PredecessorTransform and SuccessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        PositionTolerance = constraintRevoluteJoint.PositionToleranceDefault

        %JointPositionLimits Limits on the position of joint constraint (in radians)
        %   Limits on the joint position which is defined as the relative
        %   orientation between the intermediate frames defined by the
        %   PredecessorTransform and SuccessorTransform properties,
        %   respectively, along the common Z-axis. The position limits must
        %   be in the range [-pi, pi]. A zero joint position is defined
        %   with respect to the predecessor intermediate frame.
        %
        %   Default: [-pi, pi]
        JointPositionLimits = constraintRevoluteJoint.JointPositionLimitsDefault

        %OrientationTolerance Orientation tolerance of joint constraint (in radians).
        %   Tolerance on the angle between the Z-axes of the predecessor
        %   and successor intermediate frames defined by the
        %   PredecessorTransform and SuccessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        OrientationTolerance = constraintRevoluteJoint.OrientationToleranceDefault;

        %Weights Weights on value for violations of this constraint
        %
        %   Default: [1,1,1]
        Weights = constraintRevoluteJoint.WeightsDefault
    end

    properties(Access=protected)
        ConstructorProperties = {'PredecessorTransform', ...
                                 'SuccessorTransform', ...
                                 'PositionTolerance', ...
                                 'OrientationTolerance', ...
                                 'JointPositionLimits', ...
                                 'Weights'};

        ConstructorPropertyDefaultValues = ...
            {constraintRevoluteJoint.PredecessorTransformDefault, ...
             constraintRevoluteJoint.SuccessorTransformDefault, ...
             constraintRevoluteJoint.PositionToleranceDefault, ...
             constraintRevoluteJoint.OrientationToleranceDefault, ...
             constraintRevoluteJoint.JointPositionLimitsDefault, ...
             constraintRevoluteJoint.WeightsDefault};

    end

    methods
        function obj = constraintRevoluteJoint(successor, predecessor, varargin)
        %constraintRevoluteJoint Constructor
            obj.setProperties(nargin, ...
                              successor, ...
                              predecessor, ...
                              varargin{:}, ...
                              'SuccessorBody', ...
                              'PredecessorBody');
        end

        function newobj = copy(obj)
        %COPY Create copy of constraint with same property values
        %   NEWOBJ = COPY(OBJ) creates a constraintRevoluteJoint object, NEWOBJ,
        %   with the same property values as OBJ. OBJ must be a scalar
        %   handle object.
            validateattributes(obj, {'constraintRevoluteJoint'}, ...
                               {'scalar'}, 'copy', 'obj');
            newobj = constraintRevoluteJoint(obj.SuccessorBody, obj.PredecessorBody);
            newobj.SuccessorBody = obj.SuccessorBody;
            newobj.PredecessorTransform = obj.PredecessorTransform;
            newobj.SuccessorTransform = obj.SuccessorTransform;
            newobj.PositionTolerance = obj.PositionTolerance;
            newobj.OrientationTolerance = obj.OrientationTolerance;
            newobj.JointPositionLimits = obj.JointPositionLimits;
            newobj.Weights = obj.Weights;
        end

        function set.SuccessorBody(obj, successorBody)
        %set.SuccessorBody
            obj.SuccessorBody = ...
                robotics.internal.validation.validateString(...
                successorBody, false, ...
                'set.SuccessorBody', 'SuccessorBody');
        end

        function set.PredecessorBody(obj, predecessorBody)
        %set.PredecessorBody
            obj.PredecessorBody = ...
                robotics.internal.validation.validateString(...
                predecessorBody, false, ...
                'set.PredecessorBody', 'PredecessorBody');
        end

        function set.PredecessorTransform(obj, predecessorTransform)
        %set.PredecessorTransform
            robotics.internal.validation.validateHomogeneousTransform(...
                predecessorTransform, ...
                'set.PredecessorTransform', 'PredecessorTransform');
            obj.PredecessorTransform = predecessorTransform;
        end

        function set.SuccessorTransform(obj, successorTransform)
        %set.SuccessorTransform
            robotics.internal.validation.validateHomogeneousTransform(...
                successorTransform, ...
                'set.SuccessorTransform', 'SuccessorTransform');
            obj.SuccessorTransform = successorTransform;
        end

        function set.PositionTolerance(obj, positionTolerance)
        %set.PositionTolerance
            validateattributes(positionTolerance, ...
                               {'numeric'}, {'nonnegative', 'scalar', 'finite'}, ...
                               'set.PositionTolerance', 'PositionTolerance');
            obj.PositionTolerance = double(positionTolerance);
        end

        function set.JointPositionLimits(obj, jointPositionLimits)
        %set.JointPositionLimits
            validateattributes(jointPositionLimits, ...
                               {'numeric'}, ...
                               {'finite', 'row', 'ncols', 2, 'nondecreasing',...
                                   '>=', -pi, '<=', pi}, ...
                               'set.JointPositionLimits', 'JointPositionLimits');
            obj.JointPositionLimits = double(jointPositionLimits);
        end

        function set.OrientationTolerance(obj, orientationTolerance)
        %set.OrientationTolerance
            validateattributes(orientationTolerance, ...
                               {'numeric'}, {'nonnegative', 'scalar', 'finite'}, ...
                               'set.OrientationTolerance', 'OrientationTolerance');
            obj.OrientationTolerance = double(orientationTolerance);
        end

        function set.Weights(obj, weights)
        %set.Weights
            validateattributes(weights, ...
                               {'numeric'}, {'finite', 'nonnegative', 'row', 'ncols', 3}, ...
                               'set.Weights', 'Weights');
            obj.Weights = double(weights);
        end
    end
end
