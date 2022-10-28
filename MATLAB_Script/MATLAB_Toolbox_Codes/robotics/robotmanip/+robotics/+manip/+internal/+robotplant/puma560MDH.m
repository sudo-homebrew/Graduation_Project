
function [puma2, bodynames, robotnumbers] = puma560MDH()
    %puma560MDH PUMA robot model, described by modified DH.
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    bodynames = {'L1'  'L2'  'L3'  'L4'  'L5'  'L6'};
    robotnumbers = [6 6 6 6]; % numBodies, numNonFixedBodies, positionNum, velocityNum

    puma2 = rigidBodyTree('MaxNumBodies', 6);

    % L1
    jnt1 = rigidBodyJoint('jnt1','revolute');
    jnt1.setFixedTransform([0, 0, 0, 0], 'mdh'); %a alpha d theta
    jnt1.PositionLimits = [-160, 160]*pi/180;
    link1 = rigidBody('L1');
    link1.Joint = jnt1;

    puma2.addBody(link1, 'base');

    % L2
    jnt2 = rigidBodyJoint('jnt2','revolute');
    jnt2.setFixedTransform([0, -pi/2, 0.2435, 0],'mdh'); %a alpha d theta
    jnt2.PositionLimits = [-45, 225]*pi/180;
    link2 = rigidBody('L2');
    link2.Joint = jnt2;

    puma2.addBody(link2, 'L1');

    % L3
    jnt3 = rigidBodyJoint('jnt3','revolute');
    jnt3.setFixedTransform([0.4318, 0, -0.0934, 0 ], 'mdh'); %a alpha d theta
    jnt3.PositionLimits = [-225, 45]*pi/180;
    link3 = rigidBody('L3');
    link3.Joint = jnt3;

    puma2.addBody(link3, 'L2');

    % L4
    jnt4 = rigidBodyJoint('jnt4','revolute');
    jnt4.setFixedTransform([-0.0203,  pi/2, 0.4331, 0], 'mdh'); %a alpha d theta
    jnt4.PositionLimits = [-110, 170]*pi/180;
    link4 = rigidBody('L4');
    link4.Joint = jnt4;

    puma2.addBody(link4, 'L3');

    % L5
    jnt5 = rigidBodyJoint('jnt5','revolute');
    jnt5.setFixedTransform([0.0, -pi/2, 0.0, 0], 'mdh'); %a alpha d theta
    jnt5.PositionLimits = [-100, 100]*pi/180;
    link5 = rigidBody('L5');
    link5.Joint = jnt5;

    puma2.addBody(link5, 'L4');

    % L6
    jnt6 = rigidBodyJoint('jnt6','revolute');
    jnt6.setFixedTransform([0.0, pi/2, 0.0, 0], 'mdh'); %a alpha d theta
    jnt6.PositionLimits = [-266, 266]*pi/180;
    link6 = rigidBody('L6');
    link6.Joint = jnt6;

    puma2.addBody(link6, 'L5');

    %puma2.showDetails

    %puma2.show(puma2.randomConfiguration)
end
