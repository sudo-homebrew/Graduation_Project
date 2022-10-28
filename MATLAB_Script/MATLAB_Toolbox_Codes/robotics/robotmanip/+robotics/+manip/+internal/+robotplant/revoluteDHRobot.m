function robot = revoluteDHRobot(dhParams, useIntermediateFrames, worldFrame)
% This file is for internal use only and may be removed in a future release
%
%REVOLUTEDHROBOT Build a revolute robot using DH parameters
%   This file constructs a robot with M bodies from the Mx4 matrix of DH
%   parameters. Each row of the DH Parameter matrix has the format 
%   [a alpha s theta]. The function also has the following optional inputs:
%
%      useIntermediateFrames   - Use a fixed-joint frame to represent the 
%                                end of each body instead of the default
%                                where the DH frame is at the end of the
%                                body, but the joint associated with it is
%                                moving and not collocated. The default
%                                value is FALSE.
%
%      worldFrame              - Frame that maps the origin to the robot  
%                                base. This adds a second base link called
%                                frame0 that is the parent of the first
%                                moving joint. By default, this value is
%                                empty and the parent of the first moving
%                                joint is simply the base.

%   Copyright 2020 The MathWorks, Inc.

robot = rigidBodyTree;
robot.DataFormat = 'column';
robot.BaseName = 'base';

if nargin < 2
    useIntermediateFrames = false;
end

% Initialize parent. Add secondary base link if needed.
if nargin > 2 && ~isempty(worldFrame)
    % Use a rotated frame after the world
    frame0 = rigidBody('frame0');
    frame0.Joint = rigidBodyJoint('baseJoint', 'fixed');
    frame0.Joint.setFixedTransform(worldFrame);
    robot.addBody(frame0, robot.BaseName);
    parentBodyName = frame0.Name;
else
    parentBodyName = robot.BaseName;
end

% Loop through DH parameters. If using intermediate frames, split the DH
% parameters into two frames. Otherwise, just add them in bulk
for i = 1:size(dhParams,1)
    rbTheta = rigidBody(sprintf('rb%i_theta',i));
    if useIntermediateFrames
        % Add rotation about theta to the revolute joint
        thetaJoint = rigidBodyJoint(sprintf('j%i_theta',i), 'revolute');
        thetaJoint.setFixedTransform([0 0 0 0], 'dh');
        rbTheta.Joint = thetaJoint;
        
        % Add body & update parent name
        robot.addBody(rbTheta, parentBodyName);
        parentBodyName = rbTheta.Name;
        
        % Add a fixed joint to extend the body to the end (where the normal
        % DH parameter would be)
        a = dhParams(i,1);
        alpha = dhParams(i,2);
        s = dhParams(i,3);
        rbFixed = rigidBody(sprintf('rb%i_fixed', i));
        fixedJoint = rigidBodyJoint(sprintf('j%i_bodyEnd',i), 'fixed');
        fixedJoint.setFixedTransform(trvec2tform([a 0 s])*axang2tform([1 0 0 alpha]));
        rbFixed.Joint = fixedJoint;
        
        % Add body & update parent name
        robot.addBody(rbFixed, parentBodyName);
        parentBodyName = rbFixed.Name;
    else
        % Add everything to the theta joint
        thetaJoint = rigidBodyJoint(sprintf('j%i_theta',i), 'revolute');
        thetaJoint.setFixedTransform(dhParams(i,:), 'dh');
        rbTheta.Joint = thetaJoint;
        
        % Add body & update parent name
        robot.addBody(rbTheta, parentBodyName);
        parentBodyName = rbTheta.Name;
    end
end
