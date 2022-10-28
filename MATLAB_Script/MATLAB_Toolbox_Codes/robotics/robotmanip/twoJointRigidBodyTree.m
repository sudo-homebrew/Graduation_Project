function tree = twoJointRigidBodyTree(varargin)
%TWOJOINTRIGIDBODYTREE Constructs a rigidBodyTree object with two revolute joints
%   ROBOT = TWOJOINTRIGIDBODYTREE returns a rigidBodyTree
%   object with three bodies, two moving joints connected serially, and
%   geometry defined according to the following Modified Denavit
%   Hartenberg parameters:
% 
%   mdhparams = [0   0   0.1 0;
%                0.5 0   0   0; 
%                0.5 0   0   0];
%
%   ROBOT = TWOJOINTRIGIDBODYTREE(FORMAT) allows the user to define the
%   input FORMAT. The default value is 'column'.

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

narginchk(0,1)

if nargin > 0
    DataFormat = varargin{1};
    validateattributes(varargin{1}, {'char','string'}, {},'twoJointRigidBodyTree', 'DataFormat');
    DataFormat = convertStringsToChars(DataFormat);
    tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', DataFormat); 
else
    tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', 'column');
end

%Define nonzero gravity
tree.Gravity(3) = -9.81;

mdhparams = [0 0 0.1 0;
        0.5 0 0 0;
        0.5 0 0 0];

%Add body1 to rigid body tree
body1 = rigidBody('body1');
body1.Joint = rigidBodyJoint('joint1','revolute');
body1.Joint.setFixedTransform(mdhparams(1,:),'mdh');
tree.addBody(body1,tree.BaseName);

%Add body2 to rigid body tree
body2 = rigidBody('body2');
body2.Joint = rigidBodyJoint('joint2','revolute');
body2.Joint.setFixedTransform(mdhparams(2,:),'mdh');
tree.addBody(body2, 'body1');

%Add tool to rigid body tree
body3 = rigidBody('tool');
body3.Joint.setFixedTransform(mdhparams(3,:), 'mdh');
tree.addBody(body3, 'body2');

end

