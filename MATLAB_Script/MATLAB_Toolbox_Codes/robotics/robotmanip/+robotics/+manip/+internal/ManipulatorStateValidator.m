classdef ManipulatorStateValidator < robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%ManipulatorStateValidator Validator for a joint configuration of a rigid body tree
%   The validator takes an instance of a robotics.manip.internal.ManipStateSpace
%   and validates the configuration against the environment.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties
        %StateSpace The input ManipStateSpace
        StateSpace

        %Environment The collision objects in the environment
        Environment

        %ValidationDistance The resolution at which motion is validated
        ValidationDistance

        %Robot The underlying Robot of the state space
        Robot

        %IgnoreSelfCollision Flag to ignore self collisions during state
        %validation
        IgnoreSelfCollision = false
    end

    methods
        function obj = ManipulatorStateValidator(ss, env, validationDistance)
        %ManipulatorStateValidator Constructor

        %There is no need to check for validity of the input state space ss
        %as the API is internal.
            obj.StateSpace = ss;
            obj.Environment = env;
            obj.ValidationDistance = validationDistance;
            obj.Robot = obj.StateSpace.RigidBodyTree;
        end

        function valid = isStateValid(obj, state)
        %isStateValid Validate the input joint state
        %   The method checks if the input state of the rigid body tree is
        %   in collision

        %A state is valid if the robot is not in collision. Calling
        %"checkWorldCollision" and "checkSelfCollision" with "isExhaustive" as
        %false to early exit at the first encounter of collision. 
            numStates = size(state, 1);
            valid = true(numStates, 1);
            for i = 1:numStates
                tTree = obj.Robot.TreeInternal.forwardKinematics(state(i,:));
                baseTform = obj.Robot.getTransform(state(i,:),...
                    obj.Robot.BaseName);
                isColliding = (checkWorldCollision(obj.Robot.TreeInternal, tTree, baseTform, obj.Environment, false)) || ...
                                (~obj.IgnoreSelfCollision && checkSelfCollision(obj.Robot.TreeInternal, tTree, baseTform, false));
                valid(i) = ~isColliding;
            end
        end

        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
        %isMotionValid Validate the motion between two configurations
        %   The function interpolates between the input configurations and
        %   validates each state in the interpolation.  Note that this
        %   function doesn't output lastValid as in the case of a
        %   nav.StateValidator as this validator will be used for an RRT
        %   based planner

        %Assume that there is no collision and the motion is valid
            isValid = true;
            lastValid = nan(size(state1));

            %Find the ratios at which the interpolated states are obtained, and
            %check each of the interpolated states for validity.
            dist = obj.StateSpace.distance(state1, state2);
            if(dist == 0)
                isValid = obj.isStateValid(state1);
                if(isValid)
                    lastValid = state1;
                end
                return;
            end
            distances = 0:obj.ValidationDistance:dist;

            % Check for validity of the states between state1 and state2
            % (inclusive)
            ratios = [distances/dist, 1];

            interpStates = obj.StateSpace.interpolate(state1, state2, ratios);
            for i = 1:size(interpStates, 1)
                if(~obj.isStateValid(interpStates(i, :)))
                    isValid = false;
                    return;
                end
                lastValid = interpStates(i,:);
            end
        end
    end
end
