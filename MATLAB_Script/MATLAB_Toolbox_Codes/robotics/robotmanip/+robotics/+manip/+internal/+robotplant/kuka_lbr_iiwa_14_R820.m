function [robot, bodynames, robotnumbers] = kuka_lbr_iiwa_14_R820(format, maxbodycount)
%kuka_lbr_iiwa_14_R820 Constructs a KUKA light weight arm model

%   Copyright 2016-2020 The MathWorks, Inc.

%#codegen

bodynames = {};
robotnumbers = [9, 7, 7, 7]; % numBodies, numNonFixedBodies, positionNum, velocityNum


if nargin > 0
    df = format; 
else
    df = 'struct'; 
end

mb = 9;
if nargin > 1
    mb = maxbodycount; 
end

robot = rigidBodyTree('MaxNumBodies', mb, 'DataFormat', df);
robot.BaseName = 'base_link';
% Add body, 'link_1', and joint, 'joint_a1'
bodyName = 'link_1';
parentName = 'base_link';
jointName = 'joint_a1';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [               -2.9668,                 2.9668];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_2', and joint, 'joint_a2'
bodyName = 'link_2';
parentName = 'link_1';
jointName = 'joint_a2';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,            -0.00043624; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                   0.36; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      1,                      0];
jointPositionLimits = [               -2.0942,                 2.0942];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_3', and joint, 'joint_a3'
bodyName = 'link_3';
parentName = 'link_2';
jointName = 'joint_a3';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [               -2.9668,                 2.9668];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_4', and joint, 'joint_a4'
bodyName = 'link_4';
parentName = 'link_3';
jointName = 'joint_a4';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,             0.00043624; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                   0.42; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                     -1,                      0];
jointPositionLimits = [               -2.0942,                 2.0942];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_5', and joint, 'joint_a5'
bodyName = 'link_5';
parentName = 'link_4';
jointName = 'joint_a5';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [               -2.9668,                 2.9668];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_6', and joint, 'joint_a6'
bodyName = 'link_6';
parentName = 'link_5';
jointName = 'joint_a6';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                    0.4; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      1,                      0];
jointPositionLimits = [               -2.0942,                 2.0942];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'link_7', and joint, 'joint_a7'
bodyName = 'link_7';
parentName = 'link_6';
jointName = 'joint_a7';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [               -3.0541,                 3.0541];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'tool0', and joint, 'joint_a7_tool0'
bodyName = 'tool0';
parentName = 'link_7';
jointName = 'joint_a7_tool0';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                  0.126; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'base', and joint, 'base_link_base'
bodyName = 'base';
parentName = 'base_link';
jointName = 'base_link_base';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName, "MaxNumCollisions", 2);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


