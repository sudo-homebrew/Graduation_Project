function tree = sixDoFJointTree(varargin)
% This function is for internal use and may be removed in a future release.
%
%SIXDOFJOINTTREE Constructs a rigidBodyTree object with three prismatic and three revolute joints
%   ROBOT = SIXDOFJOINTTREE returns a rigidBodyTree object with three
%   prismatic joints along the X, Y, and Z axes, respectively, followed by
%   three revolute joints about X, Y, and Z. The resultant tree is akin to
%   a 6-DoF joint. Furthermore, since the prismatic joints precede the
%   revolute ones, it decouples their impact, such that the orientation of
%   the end effector is entirely governed by the last three joints, while
%   their position is entirely governed by the first three joints. The tree
%   includes an extra end effector body to make pose measurement
%   straightforward.
%
%   ROBOT = SIXDOFJOINTTREE(DATAFORMAT) allows the
%   user to define the input DATAFORMAT. The default value is 'column'.

%   Copyright 2020 The MathWorks, Inc.

narginchk(0,1)

% Use zero link lengths to ensure that this tree behaves like a 6-DoF joint
% if needed
mdhparams = zeros(7,4);

jointDetails = {...
    'prismatic', [1 0 0];...
    'prismatic', [0 1 0]; ...
    'prismatic', [0 0 1]; ...
    'revolute', [1 0 0]; ...
    'revolute', [0 1 0]; ...
    'revolute', [0 0 1]; ...
    'fixed', [0 0 1]; ...
    };

if nargin > 0
    dataFormat = validatestring(varargin{1},{'column','row','struct'},'sixDoFJointTree', 'DataFormat');
else
    dataFormat = 'column';
end
tree = rigidBodyTree('MaxNumBodies', size(mdhparams,1), 'DataFormat', dataFormat);

%Define nonzero gravity
tree.Gravity(3) = -9.81;


% Construct the tree
for i = 1:size(mdhparams,1)
    
    % Create the rigid body
    bodyName = sprintf('body%i', i);
    rBody = rigidBody(bodyName);
    
    % Create the joint, add it to the body, and assign a fixed transform
    jointName = sprintf('joint%i', i);
    rBody.Joint = rigidBodyJoint(jointName, jointDetails{i,1});
    if ~strcmp(jointDetails{i,1}, 'fixed')
        rBody.Joint.JointAxis = jointDetails{i,2};
    end
    rBody.Joint.setFixedTransform(mdhparams(i,:),'mdh');
    
    % Assign the previous body as the parent
    if i == 1
        parentBody = tree.BaseName;
    else
        parentBody = tree.BodyNames{i-1};
    end
    tree.addBody(rBody, parentBody);
end

end

