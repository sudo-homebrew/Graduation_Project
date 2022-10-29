function tree = simpleBotWithCollisions(varargin)
%SIMPLEBOTWITHCOLLISIONS Constructs a rigidBodyTree object with two revolute joints
%   This file is for internal use only, and might be removed in the future.
%
%   ROBOT = SIMPLEBOTWITHCOLLISIONS returns a rigidBodyTree
%   object with three bodies, two moving joints connected serially, and
%   geometry defined according to the following Modified Denavit
%   Hartenberg parameters:
%
%   mdhparams = [0   0   0.1 0;
%                0.5 0   0   0;
%                0.5 0   0   0];
%
%   ROBOT = SIMPLEBOTWITHCOLLISIONS(FORMAT) allows the user to define the
%   input FORMAT. The default value is 'column'.
%
%   ROBOT = SIMPLEBOTWITHCOLLISIONS(FORMAT, ROBOTCOLLISIONDATA) allows the user to
%   define the input FORMAT. The default value is 'column'. ROBOTCOLLISIONDATA
%   specifies the collision data for the base, body1, body2. If not specified
%   the default value is a struct shown below.
%    collisionData = struct(...
%        "base", struct("localTransform", trvec2tform([0.5, 0, 0]) * eul2tform([0 pi/2 0]), ...
%            "name", "base", ...
%            "collisionObj", collisionCylinder(0.05, 0.5)), ...
%        "body1", struct("localTransform", trvec2tform([0, 0.5, 0]) * eul2tform([0 0 pi/2]),...
%            "name", "body1", ...
%            "collisionObj", collisionCylinder(0.05, 0.5)), ...
%        "body2", struct("localTransform", trvec2tform([0, 0.5, 0]) * eul2tform([0 0 pi/2]), ...
%            "name", "body2", ...
%            "collisionObj", collisionCylinder(0.05, 0.5)));

%   Note: The function is not supported for codegen. Merge this file with
%   robotics.manip.internal.twoRevJointBot.m when codegen is supported for
%   checkCollision, and addCollision.

%   Copyright 2020 The MathWorks, Inc.

    narginchk(0,2)

    robotCollisionData = struct(...
        "base", struct("localTransform", trvec2tform([0.5, 0, 0]) * eul2tform([0 pi/2 0]), ...
                       "name", "base", ...
                       "collisionObj", collisionCylinder(0.05, 0.5)), ...
        "body1", struct("localTransform", trvec2tform([0, 0.5, 0]) * eul2tform([0 0 pi/2]),...
                        "name", "body1", ...
                        "collisionObj", collisionCylinder(0.05, 0.5)), ...
        "body2", struct("localTransform", trvec2tform([0, 0.5, 0]) * eul2tform([0 0 pi/2]), ...
                        "name", "body2", ...
                        "collisionObj", collisionCylinder(0.05, 0.5)));

    if nargin > 0
        if(nargin == 1)
            DataFormat = varargin{1};
            validateattributes(varargin{1}, {'char','string'}, {},'twoJointRigidBodyTree', 'DataFormat');
            DataFormat = convertStringsToChars(DataFormat);
            tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', DataFormat);
        elseif(nargin == 2)
            robotCollisionData = varargin{2};
            DataFormat = varargin{1};
            validateattributes(varargin{1}, {'char','string'}, {},'twoJointRigidBodyTree', 'DataFormat');
            DataFormat = convertStringsToChars(DataFormat);
            tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', DataFormat);
        end
    else
        tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', 'column');
    end

    %Define nonzero gravity
    tree.Gravity(3) = -9.81;

    mdhparams = [0 0 0.1 0;
                 0.5 0 0 0;
                 0.5 0 0 0];

    %Add collision data to the base
    addCollision(tree.Base,...
                 robotCollisionData.base.collisionObj,...
                 robotCollisionData.base.localTransform);

    %Add body1 to rigid body tree
    body1 = rigidBody('body1');
    body1.Joint = rigidBodyJoint('joint1','revolute');
    body1.Joint.setFixedTransform(mdhparams(1,:),'mdh');
    %Add collision robotCollisionData to the body1
    addCollision(body1,...
                 robotCollisionData.body1.collisionObj,...
                 robotCollisionData.body1.localTransform);
    tree.addBody(body1,tree.BaseName);

    %Add body2 to rigid body tree
    body2 = rigidBody('body2');
    body2.Joint = rigidBodyJoint('joint2','revolute');
    body2.Joint.setFixedTransform(mdhparams(2,:),'mdh');
    %Add collision robotCollisionData to the body2
    addCollision(body2,...
                 robotCollisionData.body2.collisionObj,...
                 robotCollisionData.body2.localTransform);
    tree.addBody(body2, 'body1');

    %Add tool to rigid body tree
    body3 = rigidBody('tool');
    body3.Joint.setFixedTransform(mdhparams(3,:), 'mdh');
    tree.addBody(body3, 'body2');

end
