classdef (Abstract) LoopClosureJointConstraint < ...
    robotics.manip.internal.KinematicConstraint
%LoopClosureJointConstraint Base class for a loop-closing joint constraint

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (Dependent, SetAccess = protected)
        KinematicPath
    end

    properties (Dependent)
        % PredecessorBody Predecessor body of the joint constraint
        PredecessorBody

        % SuccessorBody Successor body of the joint constraint
        SuccessorBody
    end

    properties(Access=private)
        % PredecessorBodyIdx Predecessor body index of the joint constraint
        %   This allows to decouple the predecessor body's dependency from the
        %   "rigidBodyTree", and still maintain the invariant that the
        %   PredecessorBody is a valid body of the "rigidBodyTree".
        PredecessorBodyIdx = 0

        % SuccessorBodyIdx Successor body index of the joint constraint
        %   This allows to decouple the successor body's dependency from the
        %   "rigidBodyTree", and still maintain the invariant that the
        %   SuccessorBody is a valid body of the "rigidBodyTree".
        SuccessorBodyIdx = 0
    end

    properties
        % PredecessorTransform Intermediate frame's transform with respect to PredecessorBody
        PredecessorTransform = eye(4)

        % SuccessorTransform Intermediate frame's transform with respect to SuccessorBody
        SuccessorTransform = eye(4)
    end

    methods
        function value = get.KinematicPath(obj) 
            value = obj.Tree.kinematicPath(obj.SuccessorBody, ...
                                           obj.PredecessorBody);
        end
    end

    methods
        function obj = LoopClosureJointConstraint(varargin)
            obj@robotics.manip.internal.KinematicConstraint(varargin{:});
            obj.PredecessorBody = obj.Tree.BaseName;
            obj.SuccessorBody = obj.Tree.BaseName;
        end

        function update(obj, other)
            obj.update@robotics.manip.internal.KinematicConstraint(other);
            obj.PredecessorBody = other.PredecessorBody;
            obj.SuccessorBody = other.SuccessorBody;
            obj.PredecessorTransform = other.PredecessorTransform;
            obj.SuccessorTransform = other.SuccessorTransform;
        end

        function set.PredecessorBody(obj, value)
            obj.PredecessorBodyIdx = validateInputBodyName(obj.Tree, value);
        end

        function value = get.PredecessorBody(obj)
            if(obj.PredecessorBodyIdx > 0)
                value = obj.Tree.Bodies{obj.PredecessorBodyIdx}.Name;
            else
                value = obj.Tree.BaseName;
            end
        end

        function set.SuccessorBody(obj, value)
            obj.SuccessorBodyIdx = validateInputBodyName(obj.Tree, value);
        end

        function value = get.SuccessorBody(obj)
            if(obj.SuccessorBodyIdx > 0)
                value = obj.Tree.Bodies{obj.SuccessorBodyIdx}.Name;
            else
                value = obj.Tree.BaseName;
            end
        end

    end

    methods (Abstract)
        [g, JTwist] = evaluateConstraintFromIntermediateTransform(obj, T)
    end

    methods(Sealed)
        function [g, J] = evaluate(obj, q)
        %evaluate Evaluate the Kinematic constraint's cost and its Jacobian
        %   "p" is the predecessor body.
        %   "s" is the successor body.
        %   "sl" is the intermediate frame on the successor body.
        %   "pl" is the intermediate frame on the predecessor body.
        %   Tp_pl is the homogeneous transform of frame "pl" with respect to frame "p".
        %   Ts_sl is the homogeneous transform of frame "sl" with respect to frame "s".
        %
        %   Note that "sl" is a body-fixed frame on body "s" (in general, it is
        %   different from body "s"'s body frame). Similarly, "pl" is a
        %   body-fixed frame on body "p" (in general, it is different from body
        %   "p"'s body frame). A loop-closing joint is defined between "sl" and
        %   "pl" frames
        %
        %                   Tp_s
        %          p -------------------> s
        %          |                      |
        %     Tp_pl|                      |Ts_sl
        %          |                      |
        %          v                      v
        %          pl==================== sl
        %             Tpl_sl:=Loop Joint
        %
            p = obj.PredecessorBody;
            s = obj.SuccessorBody;
            Tp_pl = obj.PredecessorTransform;
            Ts_sl = obj.SuccessorTransform;

            % Calculate the pose of the successor("s") with respect to
            % predecessor("p"), and the Jacobian of the successor("s") in the
            % predecessor frame("p")
            [Tp_s, Jp_s] = efficientFKAndJacobianForIK(obj.Tree, q, s, p);
            [Tpl_sl, Jpl_sl] = ...
                obj.computeTransformAndJacboianBetweenIntermediateFrames(Jp_s, Tp_s, Ts_sl, Tp_pl);

            [g, JTwist] = evaluateConstraintFromIntermediateTransform(obj, Tpl_sl);

            % We need to evaluate the rate of change of the constraint cost with
            % respect to the robot's configuration "q". Use the chain rule:=
            %   dcost/dq = dcost/dTpl_sl * dTpl_sl/dq
            J = JTwist * Jpl_sl;
        end
    end

    methods(Static)
        function [Tpl_sl, Jpl_sl] = ...
                computeTransformAndJacboianBetweenIntermediateFrames(Jp_s, Tp_s, Ts_sl, Tp_pl)
            %computeTransformAndJacboianBetweenIntermediateFrames
            %   Compute the relative transform("Tpl_sl") and Jacobian ("Jpl_sl")
            %   between frames "pl" and "sl" fixed on bodies "p" and "s",
            %   respectively. The frames "pl" and "sl" are separated from "p"
            %   and "s", respectively, by fixed transforms "Tp_pl" and "Ts_sl",
            %   respectively.
            Tpl_p = robotics.core.internal.SEHelpers.tforminvSE3(Tp_pl);
            Rpl_p= Tpl_p(1:3,1:3);

            % Position of frame "sl" with respect to frame "s"
            ts_sl = Ts_sl(1:3,end);

            % Transform of frame "sl" with respect to frame "pl"
            Tpl_sl =  Tpl_p * Tp_s * Ts_sl;

            % Position of frame "sl" with respect to frame "s" expressed in
            % frame "pl".
            tpl_s_sl = Tpl_p * Tp_s * [ts_sl; 0];

            % Express the Jacobian(angular and linear velocity) of frame "s"
            % with respect to frame "pl". Note that J := [omega; v].
            Jpl_s = [Rpl_p zeros(3); 
                        zeros(3), Rpl_p] * Jp_s;

            % The angular velocity of frame "s" with respect to "pl" frame
            omegapl_s = Jpl_s(1:3,:);

            % The linear velocity of frame "s" with respect to "pl" frame
            vpl_s = Jpl_s(4:6,:);

            % The angular velocity of frame "sl", rigidly attached to frame "s",
            % is the same as angular velocity of frame "s". 
            omegapl_sl = omegapl_s;

            % The linear velocity of "sl", rigidly attached to frame "s", is the
            % cross product of the angular velocity of the frame "sl" and the
            % position vector of "sl" with respect to "s". Note that we need the
            % velocity expressed in the frame "pl", hence, we need to express
            % the position vector with respect to frame "pl".
            vs_sl = cross(omegapl_s, repmat(tpl_s_sl(1:3), 1, size(Jp_s, 2)));

            % Jacobian of frame "sl" with respect to frame "pl".
            Jpl_sl = [omegapl_sl;
                      vpl_s + vs_sl];
        end
    end
end
