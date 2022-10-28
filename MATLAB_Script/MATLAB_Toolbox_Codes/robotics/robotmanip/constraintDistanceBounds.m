classdef constraintDistanceBounds < robotics.core.internal.mixin.SetProperties
%constraintDistanceBounds Constrain distance between origins of two bodies
%   The constraintDistanceBounds object describes a constraint on the
%   distance of one body (the end effector) relative to another body (the
%   reference body) within the same rigid body tree. This constraint is
%   satisfied if the distance, "d", of the end effector origin relative to
%   the reference body origin frame satisfies:
%
%       Bounds(1) <= d <= Bounds(2)
%
%   CD = constraintDistanceBounds(ENDEFFECTOR) returns a distance bounds
%   constraint object, CD, that represents a constraint on a body whose
%   name is given by ENDEFFECTOR.
%
%   CD = constraintDistanceBounds(ENDEFFECTOR,PropertyName=PropertyValue)
%   returns a distance bounds constraint object, CD, with each specified
%   property set to the specified value.
%
%   constraintDistanceBounds properties:
%
%   EndEffector   - Name of the end effector body
%   ReferenceBody - Name of the reference body
%   Bounds        - Bounds on end effector distance relative to reference
%                   body
%   Weights       - Weighting values for violations of this constraint
%
%
%   Example:
%       % Create Distance Bounds constraint for a body named
%       % 'tool' relative to the base frame
%       constrDist = constraintDistanceBounds('tool');
%
%       % Set bounds on the distance of the end effector origin
%       % relative to the base origin. The minimum distance should
%       % be 2 m, and the maximum should be 3 m.
%       constrDist.Bounds = [2, 3];
%
%   See also generalizedInverseKinematics, rigidBodyTree

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (Constant, Access=private)
        ReferenceBodyDefault = ''
        BoundsDefault = [0,0]
        WeightsDefault = 1
    end

    properties
        %EndEffector Name of the end effector body
        %   When this constraint is passed to a
        %   generalizedInverseKinematics solver, EndEffector must be the
        %   name of a body in the solver's rigid body tree.
        EndEffector

        %ReferenceBody Name of the reference body
        %   When this constraint is passed to a
        %   generalizedInverseKinematics solver, ReferenceBody must be
        %   either the name of a body in the solver's rigid body tree or an
        %   empty character vector, or string. An empty character vector,
        %   or string, indicates that the constraint is specified relative
        %   to the base of the rigid body tree.
        %
        %   Default: ''
        ReferenceBody = constraintDistanceBounds.ReferenceBodyDefault

        %Bounds Bounds on end effector's distance from the reference body
        %   Allowable range of distance values, specified as a two-element
        %   row vector in the form ([min,max]), between the end effector's
        %   origin and the reference body.
        %
        %   Default: [0,0]
        Bounds = constraintDistanceBounds.BoundsDefault

        %Weights Weighting values for violations of this constraint
        %
        %   Default: 1
        Weights = constraintDistanceBounds.WeightsDefault

    end

    properties (Access = protected)
        %ConstructorProperties Inherited from robotics.core.internal.mixin.SetProperties
        %   Values for these properties can be specified via name-value
        %   pairs in the constructor.
        ConstructorProperties = {'ReferenceBody', 'Bounds', 'Weights'};

        %ConstructorPropertyDefaultValues Inherited from robotics.core.internal.mixin.SetProperties
        %   Default Values for constructor properties. Used in Codegen.
        ConstructorPropertyDefaultValues = {constraintDistanceBounds.ReferenceBodyDefault, ...
                                            constraintDistanceBounds.BoundsDefault,...
                                            constraintDistanceBounds.WeightsDefault};
    end

    methods
        function obj = constraintDistanceBounds(endeffector, varargin)
        %constraintDistanceBounds Constructor
            obj.setProperties(nargin, endeffector, varargin{:}, 'EndEffector');
        end

        function newobj = copy(obj)
        %COPY Create copy of constraint with same property values
        %   NEWOBJ = COPY(OBJ) creates a constraintDistanceBounds object,
        %   NEWOBJ, with the same property values as OBJ. OBJ must be a
        %   scalar handle object.
            validateattributes(obj, {'constraintDistanceBounds'}, ...
                               {'scalar'}, 'copy', 'obj');
            newobj = constraintDistanceBounds(obj.EndEffector);
            newobj.ReferenceBody = obj.ReferenceBody;
            newobj.Bounds = obj.Bounds;
            newobj.Weights = obj.Weights;
        end

        function set.EndEffector(obj, endEffector)
            obj.EndEffector = ...
                robotics.internal.validation.validateString(...
                endEffector, ...
                false, ... %allowempty=false
                'constraintDistanceBounds', 'EndEffector');
        end

        function set.ReferenceBody(obj, referenceBody)
            obj.ReferenceBody = ...
                robotics.internal.validation.validateString(...
                referenceBody, ...
                true, ... %allowempty=true
                'constraintDistanceBounds', 'ReferenceBody');
        end

        function set.Bounds(obj, bounds)
            robotics.internal.validation.validateNumericMatrix(bounds, ...
                                                               'constraintDistanceBounds', 'Bounds', ...
                                                               'size', [1,2], 'nondecreasing', 'finite');
            obj.Bounds = double(bounds);
        end

        function set.Weights(obj, weights)
            validateattributes(weights, ...
                               {'numeric'}, {'scalar', 'nonnegative', 'finite'}, ...
                               'constraintDistanceBounds', 'Weights')
            obj.Weights = double(weights);
        end

    end

end
