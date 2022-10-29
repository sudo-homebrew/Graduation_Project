function [puma, bodynames, robotnumbers] = puma560DH() 
    %puma560MDH PUMA robot model, described by DH
    
    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    bodynames = {'L1'  'L2'  'L3'  'L4'  'L5'  'L6'};
    robotnumbers = [6 6 6 6]; % numBodies, numNonFixedBodies, positionNum, velocityNum
    
   puma = rigidBodyTree('MaxNumBodies', 6);

    % L1
    jnt = rigidBodyJoint('jnt1','revolute');
    jnt.setFixedTransform([0, pi/2, 0, 0], 'dh'); %a alpha d theta
    jnt.PositionLimits = [-160, 160]*pi/180;
    link = rigidBody('L1');
    link.Joint = jnt;

    puma.addBody(link, 'base');

    % L2
    jnt.Name = 'jnt2';
    jnt.setFixedTransform([0.4318, 0, 0, 0],'dh'); %a alpha d theta
    jnt.PositionLimits = [-45, 225]*pi/180;
    link.Name = 'L2';
    link.Joint = jnt;

    puma.addBody(link, 'L1');

    % L3
    jnt.Name = 'jnt3';
    jnt.setFixedTransform([0.0203, -pi/2, 0.15005, 0],'dh'); %a alpha d theta
    jnt.PositionLimits = [-225, 45]*pi/180;
    link.Name = 'L3';
    link.Joint = jnt;

    puma.addBody(link, 'L2');

    % L4
    jnt.Name = 'jnt4';
    jnt.setFixedTransform([0.0, pi/2, 0.4318, 0],'dh'); %a alpha d theta
    jnt.PositionLimits = [-110, 170]*pi/180;
    link.Name = 'L4';
    link.Joint = jnt;

    puma.addBody(link, 'L3');

    % L5
    jnt.Name = 'jnt5';
    jnt.setFixedTransform([0.0, -pi/2, 0.0, 0],'dh'); %a alpha d theta
    jnt.PositionLimits = [-100, 100]*pi/180;
    link.Name = 'L5';
    link.Joint = jnt;

    puma.addBody(link, 'L4');

    % L6
    jnt.Name = 'jnt6';
    jnt.setFixedTransform([0.0, 0, 0.0, 0],'dh'); %a alpha d theta
    jnt.PositionLimits = [-266, 266]*pi/180;
    link.Name = 'L6';
    link.Joint = jnt;

    puma.addBody(link, 'L5');

    %puma.showDetails

    %puma.show(puma.zeroConfiguration);

end
