classdef constraintPrismaticJoint < robotics.core.internal.mixin.SetProperties
%constraintPrismaticJoint Prismatic joint constraint between two bodies.
%   constraintPrismaticJoint defines a prismatic joint constraint between a
%   successor and a predecessor body on the rigidBodyTree. The properties
%   PredecessorTransform and SuccessorTransform define intermediate frames
%   with respect to the predecessor and successor bodies, respectively.
%   This constraint allows linear motion along the common Z-axes of the
%   intermediate frames when this constraint is satisfied. The constraint
%   is satisfied when successor body's intermediate frame's origin lies on
%   the Z-axis of predecessor body's intermediate frame and there is no
%   relative orientation between the intermediate frames.
%
%   CP = constraintPrismaticJoint(SUCCESSORBODY,PREDECESSORBODY) returns a
%   prismatic joint constraint object, CP, that represents a constraint
%   between the body named SUCCESSORBODY and the body named
%   PREDECESSORBODY.
%
%   CP = constraintPrismaticJoint(_,Name=Value) returns a prismatic joint
%   constraint CP, with each specified property set to the specified value.
%
%   constraintPrismaticJoint properties:
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
%                          meters)
%   Weights              - Weights on value for violations of this
%                          constraint
%
%   Example:
%
%       % Create a prismatic joint constraint between
%       % successor body "body1" and predecessor body "body2"
%       constrPrismatic = constraintPrismaticJoint("body1","body2");
%
%       % Define the intermediate frame on the predecessor body
%       constrPrismatic.PredecessorTransform = trvec2tform([1,0,0]);
%
%   See also constraintRevoluteJoint, constraintFixedJoint,
%   generalizedInverseKinematics, rigidBodyTree

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Constant, Access=private)
        PredecessorTransformDefault = eye(4);
        SuccessorTransformDefault = eye(4);
        PositionToleranceDefault = 0
        JointPositionLimitsDefault = [-100, 100];
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
        PredecessorTransform = constraintPrismaticJoint.PredecessorTransformDefault

        %SuccessorTransform Fixed transform of successor intermediate frame
        %   This is the fixed transform of the intermediate frame on the
        %   successor body defined with respect to the successor body.
        %
        %   Default: eye(4)
        SuccessorTransform = constraintPrismaticJoint.SuccessorTransformDefault

        %PositionTolerance Position tolerance of joint constraint (in meters).
        %   The upper bound on distance between the origin of the successor
        %   intermediate frame from the Z-axis of the predecessor intermediate
        %   frame, where the intermediate frames are defined by the
        %   SuccessorTransform and PredecessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        PositionTolerance = constraintPrismaticJoint.PositionToleranceDefault

        %JointPositionLimits Limits on the constraint's joint position (in meters)
        %   Limits on the joint position which is defined as the
        %   Z-coordinate of the successor body's intermediate frame's
        %   origin with respect to the predecessor body's intermediate
        %   frame, where the intermediate frames are defined by the
        %   SuccessorTransform and PredecessorTransform properties,
        %   respectively.
        %
        %   Default: [-100, 100]
        JointPositionLimits = constraintPrismaticJoint.JointPositionLimitsDefault

        %OrientationTolerance Orientation tolerance of joint constraint (in radians).
        %   The upper bound on the magnitude of relative orientation
        %   between successor and predecessor intermediate frames defined
        %   by the SuccessorTransform and PredecessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        OrientationTolerance = constraintPrismaticJoint.OrientationToleranceDefault;

        %Weights Weights on value for violations of this constraint
        %
        %   Default: [1,1,1]
        Weights = constraintPrismaticJoint.WeightsDefault
    end

    properties(Access=protected)
        ConstructorProperties = {'PredecessorTransform', ...
                                 'SuccessorTransform', ...
                                 'PositionTolerance', ...
                                 'OrientationTolerance', ...
                                 'JointPositionLimits', ...
                                 'Weights'};

        ConstructorPropertyDefaultValues = ...
            {constraintPrismaticJoint.PredecessorTransformDefault, ...
             constraintPrismaticJoint.SuccessorTransformDefault, ...
             constraintPrismaticJoint.PositionToleranceDefault, ...
             constraintPrismaticJoint.OrientationToleranceDefault, ...
             constraintPrismaticJoint.JointPositionLimitsDefault, ...
             constraintPrismaticJoint.WeightsDefault};

    end

    methods
        function obj = constraintPrismaticJoint(successor, predecessor, varargin)
        %constraintPrismaticJoint Constructor
            obj.setProperties(nargin, ...
                              successor, ...
                              predecessor, ...
                              varargin{:}, ...
                              'SuccessorBody', ...
                              'PredecessorBody');
        end

        function newobj = copy(obj)
        %COPY Create copy of constraint with same property values
        %   NEWOBJ = COPY(OBJ) creates a constraintPrismaticJoint object, NEWOBJ,
        %   with the same property values as OBJ. OBJ must be a scalar
        %   handle object.
            validateattributes(obj, {'constraintPrismaticJoint'}, ...
                               {'scalar'}, 'copy', 'obj');
            newobj = constraintPrismaticJoint(obj.SuccessorBody, obj.PredecessorBody);
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
                               {'numeric'}, {'finite', 'row', 'ncols', 2, 'nondecreasing'}, ...
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