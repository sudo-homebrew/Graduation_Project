function [robot, bodynames, robotnumbers  ] = rethink_baxter(format)
%rethink_baxter Constructs a Rethink Baxter robot model

%   Copyright 2016-2019 The MathWorks, Inc.

%#codegen

if nargin > 0
    df = format; 
else
    df = 'struct'; 
end

bodynames = {};
robotnumbers = [48, 15, 15, 15];% numBodies, numNonFixedBodies, positionNum, velocityNum

robot = rigidBodyTree('MaxNumBodies', 48, 'DataFormat', df);
% Add body, 'collision_head_link_1', and joint, 'collision_head_1'
bodyName = 'collision_head_link_1';
parentName = 'base';
jointName = 'collision_head_1';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                   0.11; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                   0.75; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'collision_head_link_2', and joint, 'collision_head_2'
bodyName = 'collision_head_link_2';
parentName = 'base';
jointName = 'collision_head_2';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                   0.11; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                   0.75; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_torso_itb', and joint, 'left_torso_itb_fixed'
bodyName = 'left_torso_itb';
parentName = 'torso';
jointName = 'left_torso_itb_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [   -0.9999999957076562, -9.265358966049026e-05,  4.536865349841644e-16,   -0.08896999999999999; ...
                                         -0,  4.896588860146748e-12,                      1,                0.15593; ...
                     -9.265358966049026e-05,     0.9999999957076562, -4.896588839128905e-12,               0.389125; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_torso_itb', and joint, 'right_torso_itb_fixed'
bodyName = 'right_torso_itb';
parentName = 'torso';
jointName = 'right_torso_itb_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,   -0.08896999999999999; ...
                                          0,  4.896588860146748e-12,                     -1,               -0.15593; ...
                                         -0,                      1,  4.896588860146748e-12,               0.389125; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'pedestal', and joint, 'pedestal_fixed'
bodyName = 'pedestal';
parentName = 'torso';
jointName = 'pedestal_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'head', and joint, 'head_pan'
bodyName = 'head';
parentName = 'torso';
jointName = 'head_pan';
jointType = 'revolute';
T_Joint_to_Parent = [                     1,                      0,                      0,                   0.06; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,     0.6860000000000001; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -1.57079632679,          1.57079632679];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'dummyhead1', and joint, 'dummy'
bodyName = 'dummyhead1';
parentName = 'head';
jointName = 'dummy';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'screen', and joint, 'head_nod'
bodyName = 'screen';
parentName = 'head';
jointName = 'head_nod';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12,      0.178806898652768,     0.9838841867792056,                 0.1227; ...
                                          1, -8.755438680605324e-13,   -4.8176763486576e-12,                      0; ...
                                         -0,     0.9838841867792056,     -0.178806898652768,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'display', and joint, 'display_joint'
bodyName = 'display';
parentName = 'screen';
jointName = 'display_joint';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                 -0.016; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'head_camera', and joint, 'head_camera'
bodyName = 'head_camera';
parentName = 'head';
jointName = 'head_camera';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12,      0.178806898652768,     0.9838841867792056,                0.12839; ...
                                          1, -8.755438680605324e-13,   -4.8176763486576e-12,                      0; ...
                                         -0,     0.9838841867792056,     -0.178806898652768,                0.06368; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'sonar_ring', and joint, 'sonar_s0'
bodyName = 'sonar_ring';
parentName = 'torso';
jointName = 'sonar_s0';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,    0.09470000000000001; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                  0.817; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_upper_elbow', and joint, 'right_e0'
bodyName = 'right_upper_elbow';
parentName = 'right_lower_shoulder';
jointName = 'right_e0';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.102; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -3.05417993878,          3.05417993878];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_lower_elbow', and joint, 'right_e1'
bodyName = 'right_lower_elbow';
parentName = 'right_upper_elbow';
jointName = 'right_e1';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12,                      1, -4.896588860146748e-12,    0.06900000000000001; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                          1, -4.896588860146748e-12,  2.397658246531323e-23,                0.26242; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                 -0.05,                  2.618];
jointHomePosition =                  1.284;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_upper_forearm', and joint, 'right_w0'
bodyName = 'right_upper_forearm';
parentName = 'right_lower_elbow';
jointName = 'right_w0';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                0.10359; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -3.059,                  3.059];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_arm_itb', and joint, 'right_w0_to_itb_fixed'
bodyName = 'right_arm_itb';
parentName = 'right_upper_forearm';
jointName = 'right_w0_to_itb_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                     -1,                -0.0565; ...
                                          1,  2.397658246531323e-23,  4.896588860146748e-12,                      0; ...
                                         -0,                     -1,  4.896588860146748e-12,                   0.12; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_lower_forearm', and joint, 'right_w1'
bodyName = 'right_lower_forearm';
parentName = 'right_upper_forearm';
jointName = 'right_w1';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12,                      1, -4.896588860146748e-12,                   0.01; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                          1, -4.896588860146748e-12,  2.397658246531323e-23,                 0.2707; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -1.57079632679,                  2.094];
jointHomePosition =     0.2616018366049999;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_wrist', and joint, 'right_w2'
bodyName = 'right_wrist';
parentName = 'right_lower_forearm';
jointName = 'right_w2';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,               0.115975; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -3.059,                  3.059];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_hand', and joint, 'right_hand'
bodyName = 'right_hand';
parentName = 'right_wrist';
jointName = 'right_hand';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                0.11355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_hand_camera', and joint, 'right_hand_camera'
bodyName = 'right_hand_camera';
parentName = 'right_hand';
jointName = 'right_hand_camera';
jointType = 'fixed';
T_Joint_to_Parent = [-3.205103454691839e-09,                      1,                     -0,                0.03825; ...
                                         -1, -3.205103454691839e-09,                      0,                  0.012; ...
                                         -0,                      0,                      1,               0.015355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_hand_camera_axis', and joint, 'right_hand_camera_axis'
bodyName = 'right_hand_camera_axis';
parentName = 'right_hand';
jointName = 'right_hand_camera_axis';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                0.03825; ...
                                          0,                      1,                      0,                  0.012; ...
                                         -0,                      0,                      1,               0.015355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_hand_range', and joint, 'right_hand_range'
bodyName = 'right_hand_range';
parentName = 'right_hand';
jointName = 'right_hand_range';
jointType = 'fixed';
T_Joint_to_Parent = [ 2.397658246531323e-23,                      1, -4.896588860146748e-12,                  0.032; ...
                     -4.896588860146748e-12,  4.896588860146748e-12,                      1,              -0.020245; ...
                                          1,                      0,  4.896588860146748e-12,                 0.0288; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_hand_accelerometer', and joint, 'right_hand_accelerometer'
bodyName = 'right_hand_accelerometer';
parentName = 'right_hand';
jointName = 'right_hand_accelerometer';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                0.00198; ...
                                          0,                      1,                      0,               0.000133; ...
                                         -0,                      0,                      1,                -0.0146; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_gripper_base', and joint, 'right_gripper_base'
bodyName = 'right_gripper_base';
parentName = 'right_hand';
jointName = 'right_gripper_base';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_gripper', and joint, 'right_endpoint'
bodyName = 'right_gripper';
parentName = 'right_gripper_base';
jointName = 'right_endpoint';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                  0.025; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_upper_forearm_visual', and joint, 'right_w0_fixed'
bodyName = 'right_upper_forearm_visual';
parentName = 'right_lower_elbow';
jointName = 'right_w0_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.088; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'right_upper_elbow_visual', and joint, 'right_e0_fixed'
bodyName = 'right_upper_elbow_visual';
parentName = 'right_lower_shoulder';
jointName = 'right_e0_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.107; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


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
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_upper_elbow', and joint, 'left_e0'
bodyName = 'left_upper_elbow';
parentName = 'left_lower_shoulder';
jointName = 'left_e0';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.102; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -3.05417993878,          3.05417993878];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_lower_elbow', and joint, 'left_e1'
bodyName = 'left_lower_elbow';
parentName = 'left_upper_elbow';
jointName = 'left_e1';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12,                      1, -4.896588860146748e-12,    0.06900000000000001; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                          1, -4.896588860146748e-12,  2.397658246531323e-23,                0.26242; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                 -0.05,                  2.618];
jointHomePosition =                  1.284;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_upper_forearm', and joint, 'left_w0'
bodyName = 'left_upper_forearm';
parentName = 'left_lower_elbow';
jointName = 'left_w0';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                0.10359; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -3.059,                  3.059];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_arm_itb', and joint, 'left_w0_to_itb_fixed'
bodyName = 'left_arm_itb';
parentName = 'left_upper_forearm';
jointName = 'left_w0_to_itb_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                     -1,                -0.0565; ...
                                          1,  2.397658246531323e-23,  4.896588860146748e-12,                      0; ...
                                         -0,                     -1,  4.896588860146748e-12,                   0.12; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_lower_forearm', and joint, 'left_w1'
bodyName = 'left_lower_forearm';
parentName = 'left_upper_forearm';
jointName = 'left_w1';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12,                      1, -4.896588860146748e-12,                   0.01; ...
                                          0,  4.896588860146748e-12,                      1,                      0; ...
                                          1, -4.896588860146748e-12,  2.397658246531323e-23,                 0.2707; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [        -1.57079632679,                  2.094];
jointHomePosition =     0.2616018366049999;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_wrist', and joint, 'left_w2'
bodyName = 'left_wrist';
parentName = 'left_lower_forearm';
jointName = 'left_w2';
jointType = 'revolute';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,               0.115975; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [                -3.059,                  3.059];
jointHomePosition =                      0;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_hand', and joint, 'left_hand'
bodyName = 'left_hand';
parentName = 'left_wrist';
jointName = 'left_hand';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                0.11355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_hand_camera', and joint, 'left_hand_camera'
bodyName = 'left_hand_camera';
parentName = 'left_hand';
jointName = 'left_hand_camera';
jointType = 'fixed';
T_Joint_to_Parent = [-3.205103454691839e-09,                      1,                     -0,                0.03825; ...
                                         -1, -3.205103454691839e-09,                      0,                  0.012; ...
                                         -0,                      0,                      1,               0.015355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_hand_camera_axis', and joint, 'left_hand_camera_axis'
bodyName = 'left_hand_camera_axis';
parentName = 'left_hand';
jointName = 'left_hand_camera_axis';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                0.03825; ...
                                          0,                      1,                      0,                  0.012; ...
                                         -0,                      0,                      1,               0.015355; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_hand_range', and joint, 'left_hand_range'
bodyName = 'left_hand_range';
parentName = 'left_hand';
jointName = 'left_hand_range';
jointType = 'fixed';
T_Joint_to_Parent = [ 2.397658246531323e-23,                      1, -4.896588860146748e-12,                  0.032; ...
                     -4.896588860146748e-12,  4.896588860146748e-12,                      1,              -0.020245; ...
                                          1,                      0,  4.896588860146748e-12,                 0.0288; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_hand_accelerometer', and joint, 'left_hand_accelerometer'
bodyName = 'left_hand_accelerometer';
parentName = 'left_hand';
jointName = 'left_hand_accelerometer';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                0.00198; ...
                                          0,                      1,                      0,               0.000133; ...
                                         -0,                      0,                      1,                -0.0146; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_gripper_base', and joint, 'left_gripper_base'
bodyName = 'left_gripper_base';
parentName = 'left_hand';
jointName = 'left_gripper_base';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_gripper', and joint, 'left_endpoint'
bodyName = 'left_gripper';
parentName = 'left_gripper_base';
jointName = 'left_endpoint';
jointType = 'fixed';
T_Joint_to_Parent = [                     1,                      0,                      0,                      0; ...
                                          0,                      1,                      0,                      0; ...
                                         -0,                      0,                      1,                  0.025; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_upper_forearm_visual', and joint, 'left_w0_fixed'
bodyName = 'left_upper_forearm_visual';
parentName = 'left_lower_elbow';
jointName = 'left_w0_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.088; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


% Add body, 'left_upper_elbow_visual', and joint, 'left_e0_fixed'
bodyName = 'left_upper_elbow_visual';
parentName = 'left_lower_shoulder';
jointName = 'left_e0_fixed';
jointType = 'fixed';
T_Joint_to_Parent = [ 4.896588860146748e-12, -4.896588860146748e-12,                      1,                  0.107; ...
                                          1,  2.397658246531323e-23, -4.896588860146748e-12,                      0; ...
                                         -0,                      1,  4.896588860146748e-12,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);

body = rigidBody(bodyName);
body.Joint = joint;
robot.addBody(body, parentName);bodynames{end+1} = bodyName;


