function [robot, bodynames, robotnumbers] = tree1WithLeafFixedBodies()
%This function is for internal use only. It may be removed in the future.

%tree1WithLeafFixedJoint Constructs a robot with branches like below with 6
%   revolute bodies and 2 leaf fixed bodies and 1 non leaf fixed body. 
%   All fixed bodies have positive mass and PD inertia.
%
%        b_fixed1
%           \
%            b5   b6 
%             \  /
%         b4   b3
%          \  /
%           b2
%           |
%           b1
%           |
%           b_fixed0
%           :
%          base -- b_fixed2

%   Copyright 2016-2019 The MathWorks, Inc.

%#codegen

bodynames = {'b_fixed0','b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'bfixed1', 'b_fixed2'};
robotnumbers = [9, 6, 6, 6];% numBodies, numNonFixedBodies, positionNum, velocityNum

robot = rigidBodyTree('MaxNumBodies', 6);
robot.BaseName = 'base';

% Add body 'b_fixed0'
bodyName = 'b_fixed0';
parentName = 'base';
jointName = 'j_fixed0';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  0,    0,   -1,    0; ...
                       0,    1,    0,    0; ...
                       1,    0,    0,  0.08; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b1', and joint 'j1'
bodyName = 'b1';
parentName = 'b_fixed0';
jointName = 'j1';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  1,    0,    0,  0.02; ...
                       0,   -1,    0,    0; ...
                       0,    0,   -1,    0; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b2', and joint 'j2'
bodyName = 'b2';
parentName = 'b1';
jointName = 'j2';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  1,    0,    0,  0.2; ...
                       0,    1,    0,    0; ...
                       0,    0,    1,    0; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b3', and joint 'j3'
bodyName = 'b3';
parentName = 'b2';
jointName = 'j3';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  1,    0,    0,  0.2; ...
                       0,    1,    0,    0; ...
                       0,    0,    1,    0; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b4', and joint 'j4'
bodyName = 'b4';
parentName = 'b2';
jointName = 'j4';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  0,    -1,   0,  0.1; ...
                       0,    0,   -1,    0; ...
                       1,    0,    0,  0.1; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b5', and joint 'j5'
bodyName = 'b5';
parentName = 'b3';
jointName = 'j5';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  0,   -1,    0,  0.1; ...
                       1,    0,    0,  0.1; ...
                       0,    0,    1,    0; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b6', and joint 'j6'
bodyName = 'b6';
parentName = 'b3';
jointName = 'j6';
jointType = 'revolute';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  cos(pi/4), cos(pi/4),    0,  0.2; ...
                      -sin(pi/4), sin(pi/4),    0, -0.1; ...
                               0,         0,    1,    0; ...
                               0,         0,    0,    1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);



% Add body 'b_fixed1'
bodyName = 'b_fixed1';
parentName = 'b5';
jointName = 'j_fixed1';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  0,    1,    0,  0.2; ...
                      -1,    0,    0,    0; ...
                       0,    0,    1,    0; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);


% Add body 'b_fixed2'
bodyName = 'b_fixed2';
parentName = 'base';
jointName = 'j_fixed2';
mass = 1;
com = [0.1, 0, 0];
%assuming slender rod and uniform density
izz = (1/3)*mass*0.2^2;
iyy = izz;
inertia = [1e-8 iyy izz 0 0 0];
T_Joint_to_Parent = [  0,   -1,    0,    0; ...
                       1,    0,    0,   0.1; ...
                       0,    0,    1,  0.1; ...
                       0,    0,    0,    1];

joint = rigidBodyJoint(jointName);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = com;
body.Inertia = inertia;
robot.addBody(body, parentName);
