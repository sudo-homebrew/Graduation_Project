classdef constraintFixedJoint < robotics.core.internal.mixin.SetProperties
%constraintFixedJoint Fixed joint constraint between two bodies.
%   constraintFixedJoint defines a fixed joint constraint between a
%   successor and a predecessor body on the rigidBodyTree. An
%   intermediate frame is defined each with respect to the predecessor and
%   successor body, defined by the properties PredecessorTransform and
%   SuccessorTransform, respectively. This constraint allows no relative
%   motion between the intermediate frames when satisfied. The constraint
%   is satisfied when the origin of the successor body's intermediate frame
%   coincides with origin of predecessor body's intermediate frame and
%   there is no relative orientation between the intermediate frames.
%
%   CF = constraintFixedJoint(SUCCESSORBODY,PREDECESSORBODY) returns a
%   fixed joint constraint object, CF, that represents a constraint
%   between the body named SUCCESSORBODY and the body named
%   PREDECESSORBODY.
%
%   CF = constraintFixedJoint(_,Name=Value) returns a fixed joint
%   constraint CF, with each specified property set to the specified value.
%
%   constraintFixedJoint properties:
%
%   SuccessorBody        - Name of joint constraint's successor body
%   PredecessorBody      - Name of joint constraint's predecessor body
%   PredecessorTransform - Transform of predecessor intermediate frame
%   SuccessorTransform   - Transform of successor intermediate frame
%   PositionTolerance    - Position tolerance of joint constraint (in
%                          meters)
%   OrientationTolerance - Orientation tolerance of joint constraint (in
%                          radians)
%   Weights              - Weights on value for violations of this
%                          constraint
%
%   Example:
%
%       % Create a fixed joint constraint between
%       % successor body "body1" and predecessor body "body2"
%       constrFixed = constraintFixedJoint("body1","body2");
%
%       % Define the intermediate frame on the predecessor body
%       constrFixed.PredecessorTransform = trvec2tform([1,0,0]);
%
%   See also constraintRevoluteJoint, constraintPrismaticJoint,
%   generalizedInverseKinematics, rigidBodyTree

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Constant, Access=private)
        PredecessorTransformDefault = eye(4);
        SuccessorTransformDefault = eye(4);
        PositionToleranceDefault = 0
        OrientationToleranceDefault = 0
        WeightsDefault = [1,1]
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

        %PredecessorTransform Fixed transform of predecessor's intermediate frame
        %   This is the fixed transform of the intermediate frame on the
        %   predecessor body defined with respect to the predecessor body
        %   frame. By default, the intermediate frame coincides with the
        %   body frame.
        %
        %   Default: eye(4)
        PredecessorTransform = constraintFixedJoint.PredecessorTransformDefault

        %SuccessorTransform Fixed transform of successor's intermediate frame
        %   This is the fixed transform of the intermediate frame on the
        %   successor body defined with respect to the successor body. By
        %   default, the intermediate frame coincides with the body
        %   frame.
        %
        %   Default: eye(4)
        SuccessorTransform = constraintFixedJoint.SuccessorTransformDefault

        %PositionTolerance Position tolerance of joint constraint (in meters).
        %   Tolerance on the distance between the origins of the
        %   predecessor and successor intermediate frames defined by the
        %   PredecessorTransform and SuccessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        PositionTolerance = constraintFixedJoint.PositionToleranceDefault

        %OrientationTolerance Orientation tolerance of joint constraint (in radians).
        %   The upper bound on the magnitude of relative orientation
        %   between successor and predecessor intermediate frames defined
        %   by the PredecessorTransform and SuccessorTransform properties,
        %   respectively.
        %
        %   Default: 0
        OrientationTolerance = constraintFixedJoint.OrientationToleranceDefault;

        %Weights Weights on value for violations of this constraint
        %
        %   Default: [1,1]
        Weights = constraintFixedJoint.WeightsDefault
    end

    properties(Access=protected)
        ConstructorProperties = {'PredecessorTransform', ...
                                 'SuccessorTransform', ...
                                 'PositionTolerance', ...
                                 'OrientationTolerance', ...
                                 'Weights'};

        ConstructorPropertyDefaultValues = ...
            {constraintFixedJoint.PredecessorTransformDefault, ...
             constraintFixedJoint.SuccessorTransformDefault, ...
             constraintFixedJoint.PositionToleranceDefault, ...
             constraintFixedJoint.OrientationToleranceDefault, ...
             constraintFixedJoint.WeightsDefault};

    end

    methods
        function obj = constraintFixedJoint(successor, predecessor, varargin)
        %constraintFixedJoint Constructor
            obj.setProperties(nargin, ...
                              successor, ...
                              predecessor, ...
                              varargin{:}, ...
                              'SuccessorBody', ...
                              'PredecessorBody');
        end

        function newobj = copy(obj)
        %COPY Create copy of constraint with same property values
        %   NEWOBJ = COPY(OBJ) creates a constraintFixedJoint object, NEWOBJ,
        %   with the same property values as OBJ. OBJ must be a scalar
        %   handle object.
            validateattributes(obj, {'constraintFixedJoint'}, ...
                               {'scalar'}, 'copy', 'obj');
            newobj = constraintFixedJoint(obj.SuccessorBody, obj.PredecessorBody);
            newobj.SuccessorBody = obj.SuccessorBody;
            newobj.PredecessorTransform = obj.PredecessorTransform;
            newobj.SuccessorTransform = obj.SuccessorTransform;
            newobj.PositionTolerance = obj.PositionTolerance;
            newobj.OrientationTolerance = obj.OrientationTolerance;
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
                               {'numeric'}, {'finite', 'nonnegative', 'row', 'ncols', 2}, ...
                               'set.Weights', 'Weights');
            obj.Weights = double(weights);
        end
    end
end
