function [rob, bodynames, robotnumbers] = fixedbot(  )
    %FIXEDBOT Constructs a simple robot with only fixed joints just for testing

    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen

    bodynames = {'b1'  'b2'  'tool' };
    robotnumbers = [3 0 0 0]; % numBodies, numNonFixedBodies, positionNum, velocityNum
    
    rob = rigidBodyTree('MaxNumBodies', 3);
    body1 = rigidBody('b1');
    body1.Joint.setFixedTransform([0 0 0.1 -pi/4],'mdh');
    rob.addBody(body1,rob.BaseName);
    
    body2 = rigidBody('b2');
    body2.Joint.setFixedTransform([0.5 0 0 pi/2],'mdh');
    rob.addBody(body2, 'b1');
    
    body3 = rigidBody('tool');
    body3.Joint.setFixedTransform([0.5 0 0 0], 'mdh');
    rob.addBody(body3, 'b2');
end

