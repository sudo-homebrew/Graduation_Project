function [rob, bodynames, robotnumbers] = simplebot(varargin)
    %SIMPLEBOT Constructs a very simple robot for testing purpose only

    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen

    bodynames = {'b1'  'b2'  'tool'};
    robotnumbers = [3 2 2 2]; % numBodies, numNonFixedBodies, positionNum, velocityNum
    

    if nargin > 0
        df = varargin{1}; 
    else
        df = 'struct';
    end
    
    rob = rigidBodyTree('MaxNumBodies', 3, 'DataFormat', df);
    body1 = rigidBody('b1');
    body1.Joint = rigidBodyJoint('j1','rev');
    body1.Joint.setFixedTransform([0 0 0.1 0],'mdh');
    rob.addBody(body1,rob.BaseName);
    
    body2 = rigidBody('b2');
    body2.Joint = rigidBodyJoint('j2','revolute');
    body2.Joint.setFixedTransform([0.5 0 0 0],'mdh');
    rob.addBody(body2, 'b1');
    
    body3 = rigidBody('tool');
    body3.Joint.setFixedTransform([0.5 0 0 0], 'mdh');
    rob.addBody(body3, 'b2');
end

