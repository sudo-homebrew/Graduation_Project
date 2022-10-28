classdef RigidBodyTreeUtils < robotics.manip.internal.InternalAccess
%RigidBodyTreeUtils A utility class for rigidBodyTree
%   The class offers utility functions that require accessing the internal
%   RigidBodyTree.

%   Copyright 2020-2022 The MathWorks, Inc.
%#codegen

    methods(Static)

        function [numNonFixedJoints, jointBounds] = getConfigurationInfo(robot)
        %getConfigurationInfo Get the configuration info of a rigidBodyTree
        %   The function outputs the number of non-fixed joints that make up the
        %   configuration of a rigid body tree, the joint limits of the non-fixed
        %   joints.
            numNonFixedJoints = robot.TreeInternal.PositionNumber;
            jointBounds = robot.TreeInternal.JointPositionLimits;
        end

        function isRevolute = identifyRevoluteJoint(robot)
        %identifyRevoluteJoint Indicates if a dimension in the configuration corresponds to a revolute joint
        %   The function returns a logical row vector that indicates if a
        %   dimension of the configuration corresponds to a revolute joint

            isRevolute = false(1, robot.TreeInternal.PositionNumber);
            numBodies = robot.NumBodies;
            bodies = robot.Bodies;
            for i = 1 : numBodies
                bodyI = bodies{i};
                joint = bodyI.Joint;
                if(strcmp(joint.Type, 'revolute'))
                    idx = robot.TreeInternal.PositionDoFMap(i, 2);
                    isRevolute(idx) = true;
                end
            end
        end

        function dist = distance(config1, config2, isRevolute)
        %distance Distance between two configurations
        %   The computed distance between CONFIG1 and CONFIG2 is a Euclidean
        %   norm of difference between a revolute joint's values which is
        %   then wrapped to [-pi, pi], and a displacement between a
        %   prismatic joint's values. CONFIG1 and CONFIG2 should be matrices
        %   of compatible sizes. When CONFIG1 and CONFIG2 are single
        %   configurations, the output DIST is a scalar. When either of them
        %   are r-by-N matrices (where r is the number of configurations,
        %   and N is the number of non-fixed joints), DIST is a column
        %   vector of size r. ISREVOLUTE is a logical row vector that is
        %   used to define which among the N dimensions of the configuration
        %   correspond to a a revolute joint.

            if(size(config1, 1) > size(config2, 1))
                configDiff = repmat(config2, size(config1, 1), 1) - config1;
            elseif(size(config1, 1) < size(config2, 1))
                configDiff = config2 - repmat(config1, size(config2, 1), 1);
            else
                configDiff = config2 - config1;
            end

            %Wrap the configDiff for revolute joint values
            configDiff(:, isRevolute) = ...
                robotics.internal.wrapToPi(configDiff(:, isRevolute));
            dist = vecnorm(configDiff')';
        end

        function validateTreeWithMovingJoints(robot)
            %validateTreeWithMovingJoints Validate rigid body tree with non-fixed joints
            %   This method verifies that the input is a rigid body tree
            %   with at least 1 non-fixed joint. This verification is
            %   necessary for a number of functions such as IK, but
            %   requires internal access.

            if robot.NumNonFixedBodies == 0
                robotics.manip.internal.error('inversekinematics:RigidBodyTreeFixed'); 
            end           
        end
    end
end
