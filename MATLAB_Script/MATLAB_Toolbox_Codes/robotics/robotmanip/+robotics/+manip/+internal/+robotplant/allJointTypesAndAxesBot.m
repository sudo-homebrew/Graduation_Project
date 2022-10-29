function tree = allJointTypesAndAxesBot(varargin)
% This function is for internal use and may be removed in a future release.
%
%ALLJOINTTYPESANDAXESBOT Constructs a rigidBodyTree object with none joints, incl. three joints of each type
%   ROBOT = ALLJOINTTYPESANDAXESBOT returns a rigidBodyTree
%   object with three prismatic, three revolute, and three fixed joints,
%   alternating joint types in series. The robot also varies joint axes,
%   using the X-axis for the first two non-fixed joints, the Y-axis for the
%   second two, and the Z-axis for the final two non-fixed joints. The main
%   aim of the robot is to be able to provide a robot with all the major
%   construction variations. The geometry is defined according to the
%   following Modified Denavit Hartenberg parameters:
% 
% d = 0.1; %link length
% mdhparams = [...
%              0   0 d 0; ...
%              0.5 d 0 0; ...
%              0.5 0 0 d; ...
%              0   0 d 0; ...
%              0.5 d 0 0; ...
%              0.5 0 0 d; ...
%              0   0 d 0; ...
%              0.5 d 0 0; ...
%              0.5 0 0 d; ...
%              ];
%
%   ROBOT = ALLJOINTTYPESANDAXESBOT(JOINTDIRECTION) allows the user to define the
%   axes direction as a string. This may be POSITIVE or NEGATIVE.

%   ROBOT = ALLJOINTTYPESANDAXESBOT(JOINTDIRECTION, DATAFORMAT) allows the
%   user to define the input DATAFORMAT. The default value is 'column'.

%   Copyright 2019 The MathWorks, Inc.

narginchk(0,2)

dirCoeff = 1;
if nargin > 0
    jointDirection = varargin{1};
    validatestring(jointDirection, {'positive','negative'},'allJointTypesAndAxesBot', 'jointDirection');
    jointDirection = convertStringsToChars(jointDirection);
    if strcmp(jointDirection, 'negative')
        dirCoeff = -1;
    end
end

if nargin > 1
    DataFormat = varargin{2};
    validateattributes(varargin{2}, {'char','string'}, {},'allJointTypesAndAxesBot', 'DataFormat');
    DataFormat = convertStringsToChars(DataFormat);
    tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', DataFormat); 
else
    tree = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', 'column');
end

%Define nonzero gravity
tree.Gravity(3) = -9.81;

d = 0.1; %link length
mdhparams = [...
             0   0 d 0; ...
             0.5 d 0 0; ...
             0.5 0 0 d; ...
             0   0 d 0; ...
             0.5 d 0 0; ...
             0.5 0 0 d; ...
             0   0 d 0; ...
             0.5 d 0 0; ...
             0.5 0 0 d; ...
             ];
         
jointDetails = {...
    'revolute', [1 0 0];...
    'prismatic', [1 0 0]; ...
    'fixed', [0 0 0]; ...
    'revolute', [0 1 0]; ...
    'prismatic', [0 1 0]; ...
    'fixed', [0 0 0]; ...
    'revolute', [0 0 1]; ...
    'prismatic', [0 0 1]; ...
    'fixed', [0 0 0] ...
    };


% Construct the tree
for i = 1:size(mdhparams,1)
    
    % Create the rigid body
    bodyName = sprintf('body%i', i);
    rBody = rigidBody(bodyName);
    
    % Create the joint, add it to the body, and assign a fixed transform
    jointName = sprintf('joint%i', i);
    rBody.Joint = rigidBodyJoint(jointName, jointDetails{i,1});
    if ~strcmp(jointDetails{i,1}, 'fixed')
        rBody.Joint.JointAxis = jointDetails{i,2}*dirCoeff;
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

