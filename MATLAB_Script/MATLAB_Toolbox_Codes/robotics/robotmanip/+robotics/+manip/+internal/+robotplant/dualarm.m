function [robot, bodynames, robotnumbers] = dualarm(varargin)
%DUALARM Constructs a very simple two-arm manipulator (2 DoF each arm)

%   Copyright 2016-2019 The MathWorks, Inc.

%#codegen

bodynames = {'torso', 'right_arm_mount', 'right_upper_shoulder', 'right_lower_shoulder', 'left_arm_mount', 'left_upper_shoulder', 'left_lower_shoulder'};
robotnumbers = [7, 4, 4, 4];% numBodies, numNonFixedBodies, positionNum, velocityNum


if nargin > 0
    df = varargin{1}; 
else
    df = 'struct';
end

robot = rigidBodyTree('MaxNumBodies', 7, 'DataFormat', df);
robot.BaseName = 'base';


% Add body, 'torso', and joint, 'torso_t0'
bodyName = 'torso';
parentName = 'base';
jointName = 'torso_t0';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'right_arm_mount', and joint, 'right_torso_arm_mount'
bodyName = 'right_arm_mount';
parentName = 'torso';
jointName = 'right_torso_arm_mount';
jointType = 'fixed';
T_Joint_to_Parent = [    0.7071054825112363,     0.7071080798594736,                      0,               0.024645; ...
                        -0.7071080798594736,     0.7071054825112363,                     -0,              -0.219645; ...
                                         -0,                      0,                      1,               0.118588; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'right_upper_shoulder', and joint, 'right_s0'
bodyName = 'right_upper_shoulder';
parentName = 'right_arm_mount';
jointName = 'right_s0';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,               0.055695; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,               0.011038; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -1.70167993878,          1.70167993878];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'right_lower_shoulder', and joint, 'right_s1'
bodyName = 'right_lower_shoulder';
parentName = 'right_upper_shoulder';
jointName = 'right_s1';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                     -0,                      0,    0.06900000000000001; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                         -0,                     -1,  4.896588860146748e-12,                0.27035; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -2.147,                  1.047];
jointHomePosition =    -0.5499999999999999;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'left_arm_mount', and joint, 'left_torso_arm_mount'
bodyName = 'left_arm_mount';
parentName = 'torso';
jointName = 'left_torso_arm_mount';
jointType = 'fixed';
T_Joint_to_Parent = [    0.7071054825112363,    -0.7071080798594736,                      0,               0.024645; ...
                         0.7071080798594736,     0.7071054825112363,                      0,               0.219645; ...
                                         -0,                      0,                      1,               0.118588; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'left_upper_shoulder', and joint, 'left_s0'
bodyName = 'left_upper_shoulder';
parentName = 'left_arm_mount';
jointName = 'left_s0';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,               0.055695; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,               0.011038; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -1.70167993878,          1.70167993878];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


% Add body, 'left_lower_shoulder', and joint, 'left_s1'
bodyName = 'left_lower_shoulder';
parentName = 'left_upper_shoulder';
jointName = 'left_s1';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                     -0,                      0,    0.06900000000000001; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                         -0,                     -1,  4.896588860146748e-12,                0.27035; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -2.147,                  1.047];
jointHomePosition =    -0.5499999999999999;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);


