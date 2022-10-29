function robot = revoluteDHCompatibleRobot(dhParams, zscrews, useIntermediateFrames, worldFrame, jointAxes, eeTform)
% This file is for internal use only and may be removed in a future release
%
%REVOLUTEDHCOMPATIBLEROBOT Build a revolute-joint only robot from extended DH parameters using the alternative internal representation (i.e. setting JointToParentTransform instead of ParentToChildTransform)
%   This function constructs a robot with M bodies using the extended DH parameters, which includes both the Mx4 matrix of
%   DH parameters, as well as an optional (M+1)x2 matrix of ZScrews. Each
%   row of the DH Parameter matrix has the format [a alpha s theta].
%   However, the function uses the JointToParent property of the joint,
%   rather than the ChildToJoint property used by the DH parameters. This
%   creates new rigidBodyTrees that are kinematically equivalent to a
%   dhRobot with the given parameters. This means that for a joint
%   configuration, the same end effector pose can be achieved with the any
%   kinematically equivalent robots as long as the mappings -- the zscrew
%   theta offsets and the worldToBase transforms -- are known (see Example
%   2 below for reference). Note, however, that the joint frames will not
%   necessarily be collocated with those in the original representation.
%   This occurs for two reasons: (1) a robot created using
%   setTransform('dh') has frames at the end of the body, which only exist
%   in this robot when useIntermediateFrames is true, and (2) the DH
%   supported frames are sometimes positioned at a place other than the
%   physical joint location (if this is the case, the corresponding element
%   in the first column of the zscrews will be nonzero). This function is
%   predominantly constructed for use in test.
%
%   The function also has the following optional inputs:
%
%      zscrews                 - This is an (M+1)x2 matrix of values
%                                indicating the position of the desired
%                                (i.e. source frame, typically, as a robot
%                                in this context is generally built using
%                                an existing reference) robot frame
%                                relative to its DH representation. Each
%                                row has two elements, where the first
%                                element indicates the translation offset
%                                along the Z axis of the desired frame from
%                                its DH counterpart, while the second
%                                element indicates the rotational offset
%                                about theta. This function at present does
%                                not take the first element into account,
%                                but if the second element is nonzero, the
%                                home position of that joint will be
%                                altered to take it into account. This
%                                behavior is designed for use in test,
%                                which is the primary use for this internal
%                                function.
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
%                                moving joint. When the number of inputs is
%                                less than two, the parent maps directly to
%                                the base.
%
%      jointAxes               - Specifies which joint axis to use as the
%                                axis of rotation. The default value is [0
%                                0 1]. When a value other than [0 0 1] is
%                                used, the worldFrame is modified so the
%                                final robot as the same orientation as a
%                                standard DH robot with the same
%                                parameters.
%
%      eeTform                 - Frame that maps the end effector frame
%                                from the value computed using DH
%                                transforms alone to a new frame. When
%                                specified, this is added to the last frame
%                                as an additional fixed frame.
%
%   References for the ZScrew format:
%      Bin Hammam, GM (2014). Whole-Body Motion Retargeting for Humanoids
%         [PhD Thesis]. The Ohio State University.
%
%   Example 1:
%      a = 0.5;
%      s = 0.5;
%      dhParams = [repmat([a pi/2 s 0], 3, 1); repmat([0 0 s 0], 3, 1)];
%
%      % Create a robot with standard DH Z-axes
%      robotWithZAxes = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(dhParams);
%
%      % Create comparison robots that use X and Y axes for the joints
%      robotWithXAxes = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(dhParams, zeros(7,2), false, eye(4), [1 0 0]);
%      robotWithYAxes = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(dhParams, zeros(7,2), false, eye(4), [0 1 0]);
%
%      % Show the robots side-by-side
%      figure;
%      show(robotWithZAxes);
%      figure;
%      show(robotWithXAxes);
%      figure;
%      show(robotWithYAxes);
%
%      % Get the pose of the final body in the world frame and observe that
%      % all three poses match
%      eePoseZ = getTransform(robotWithZAxes, robotWithZAxes.homeConfiguration, robotWithZAxes.BodyNames{end});
%      eePoseX = getTransform(robotWithXAxes, robotWithXAxes.homeConfiguration, robotWithXAxes.BodyNames{end});
%      eePoseY = getTransform(robotWithYAxes, robotWithYAxes.homeConfiguration, robotWithYAxes.BodyNames{end});
%
%   Example 2:
%      robot = loadrobot('yaskawaMotomanMH5','DataFormat','column');
%      [~, conversionDetails] = robotics.manip.internal.AnalyticalIKHelpers.checkForDHConversionCompatibility(robot, sqrt(eps));
%       
%      % Create a robot from the DH parameters. This robot has a first
%      % joint that is not collocated with the world frame, so a
%      % worldToDHBase frame must also be extracted.'
%      robotDH = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(conversionDetails.DHParams, conversionDetails.ZScrews);
%      baseTform = conversionDetails.WorldToDHBase;
%
%      % Observe that the robot poses match at the home configurations
%      expEEPose = robot.getTransform(robot.homeConfiguration, robot.Bodies{end}.Name);
%      expEEPoseDH = baseTform*robotDH.getTransform(robotDH.homeConfiguration, robotDH.Bodies{end}.Name);
%
%      % Observe that to get the same end effector poses, it is simply
%      % necessary to offset the DH robot by the theta offsets prescribed in
%      % the ZScrews
%      expConfig = robot.randomConfiguration;
%      expEEPose = robot.getTransform(expConfig, robot.Bodies{end}.Name);
%      expEEPoseDH = baseTform*robotDH.getTransform(expConfig + conversionDetails.ZScrews(1:6,2), robotDH.Bodies{end}.Name);

%   Copyright 2020-2021 The MathWorks, Inc.

robot = rigidBodyTree;
robot.DataFormat = 'column';
robot.BaseName = 'base';

if nargin < 6
    % Transform that relates last fixed frame to EE frame
    eeTform = eye(4);
end

% Set joint axes
if nargin < 5
    jointAxes = [0 0 1];
end
rotmFromZToAxes = getRotmFromZToAxes(jointAxes);
tformFromAxesToZ = [rotmFromZToAxes' zeros(3,1); 0 0 0 1];

if nargin < 3
    useIntermediateFrames = false;
end

if nargin < 2
    zscrews = zeros(size(dhParams,1),2);
end

% Initialize parent
if nargin > 3
    % Use a rotated frame after the world
    frame0 = rigidBody('frame0');
    frame0.Joint = rigidBodyJoint('baseJoint', 'fixed');
    frame0.Joint.setFixedTransform(worldFrame*tformFromAxesToZ);
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
        thetaJoint.JointAxis = jointAxes; % Rotation about the axis
        rbTheta.Joint = thetaJoint;
        
        % Update the joint home position
        rbTheta.Joint.HomePosition = zscrews(i,2);
        
        % Add body & update parent name
        robot.addBody(rbTheta, parentBodyName);
        parentBodyName = rbTheta.Name;
        
        % Add a fixed joint to extend the body to the end (where the normal
        % DH parameter would be)
        jointDHParams = dhParams(i,:);
        rbFixed = rigidBody(sprintf('rb%i_fixed', i));
        fixedJoint = createDHCompatibleJoint(sprintf('j%i_bodyEnd',i), 'fixed', jointDHParams, rotmFromZToAxes);
        
        % If the body is the last body, some modifications are necessary to
        % ensure the end effector frame is properly captured
        if i == size(dhParams,1)
            % Ensure that the even if the joint axis is not z, the robot is
            % parameterized in the same way and the end effector
            % orientation matches that of a standard robot
            fixedJoint.setFixedTransform(fixedJoint.JointToParentTransform*rotm2tform(rotmFromZToAxes));
            
            % Add end effector transform if provided
            fixedJoint.setFixedTransform(fixedJoint.JointToParentTransform*eeTform);
        end
        
        % Apply the fixed transform to the joint
        rbFixed.Joint = fixedJoint;
        
        % Add body & update parent name
        robot.addBody(rbFixed, parentBodyName);
        parentBodyName = rbFixed.Name;
    else
        % Add everything to the theta joint
        if i > 1
            jointDHParams = dhParams(i-1,:);
        else
            jointDHParams = zeros(1,4);
        end
        thetaJoint = createDHCompatibleJoint(sprintf('j%i_theta',i), 'revolute', jointDHParams, rotmFromZToAxes);
        
        % Update the joint home position
        thetaJoint.HomePosition = zscrews(i,2);
        
        % Assign to body
        rbTheta.Joint = thetaJoint;
        
        % Add body & update parent name
        robot.addBody(rbTheta, parentBodyName);
        parentBodyName = rbTheta.Name;
        
        if i == size(dhParams,1)
            % Add a fixed joint to extend the body to the end (where the
            % normal DH parameter would be)
            rbFixed = rigidBody(sprintf('EE_link%i', i));
            fixedJoint = createDHCompatibleJoint(sprintf('j%i_bodyEnd',i), 'fixed', dhParams(i,:), rotmFromZToAxes);
            
            % Ensure that the even if the joint axis is not z, the robot is
            % parameterized in the same way and the end effector
            % orientation matches that of a standard robot
            fixedJoint.setFixedTransform(fixedJoint.JointToParentTransform*rotm2tform(rotmFromZToAxes));
            
            % Add end effector transform if it exists
            fixedJoint.setFixedTransform(fixedJoint.JointToParentTransform*eeTform);
            
            % Apply to joint
            rbFixed.Joint = fixedJoint;
            
            % Add body & update parent name
            robot.addBody(rbFixed, parentBodyName);
            parentBodyName = rbFixed.Name;
        end
    end
end
end

function joint = createDHCompatibleJoint(jointName, jointType, dhParams, tformFromAxesToZ)
    

a = dhParams(1);
alpha = dhParams(2);
s = dhParams(3);
joint = rigidBodyJoint(jointName, jointType);
if ~strcmpi(jointType, 'fixed')
    joint.JointAxis = tformFromAxesToZ*[0; 0; 1];
end

% Map the standard DH transform to one that works with the specified joint
% axis
frameTranslation = tformFromAxesToZ*[a; 0; s];
frameAlphaRotationAxis = tformFromAxesToZ*[1 0 0]';
    
% Update the joint
joint.setFixedTransform(trvec2tform(frameTranslation(:)')*axang2tform([frameAlphaRotationAxis(:)' alpha]));

end

function rotMat = getRotmFromZToAxes(jointAxis)
%getRotmFromZToAxes Get transform that maps Z axis to joint axis
%   Returns the rotation matrix that rotates the Z axis into the specified
%   one:
%      rotMat*[0; 0; 1] = jointAxis'
%   This also implies that the joint axis and translations are mapped to
%   their DH standards:
%      jointAxis*rotMat = [0 0 1];
%      linkTranslations*rotMat = [a 0 d];
%      frameRotationAxis*rotMat = [1 0 0];

    if all(jointAxis == [1 0 0])
        % Positive X axis: jointAxis = [1 0 0]
        %    frameTranslation = [s a 0];
        %    frameRotationAxis = [0 1 0];
        
        rotMat = [0 0 1; 1 0 0; 0 1 0];

    elseif all(jointAxis == [-1 0 0])
        % Negative X axis: jointAxis = [-1 0 0]
        %    frameTranslation = [-s 0 a];
        %    frameRotationAxis = [0 0 1];
        
        rotMat = [0 0 -1; 0 1 0; 1 0 0];

    elseif all(jointAxis == [0 1 0])
        % Positive Y axis: jointAxis = [0 1 0]
        %    frameTranslation = [0 s a];
        %    frameRotationAxis = [0 0 1];
        
        rotMat = [0 1 0; 0 0 1; 1 0 0];

    elseif all(jointAxis == [0 -1 0])
        % Negative Y axis: jointAxis = [0 -1 0]
        %    frameTranslation = [a -s 0];
        %    frameRotationAxis = [1 0 0];
        
        rotMat = [1 0 0; 0 0 -1; 0 1 0];

    elseif all(jointAxis == [0 0 1])
        % Positive Z axis: jointAxis = [0 0 1]
        %    frameTranslation = [a 0 s];
        %    frameRotationAxis = [1 0 0];
        
        rotMat = eye(3);

    elseif all(jointAxis == [0 0 -1])
        % Negative Z axis: jointAxis = [0 0 -1]
        %    frameTranslation = [0 a -s];
        %    frameRotationAxis = [0 1 0];
        
        rotMat = [0 1 0; 1 0 0; 0 0 -1];

    else
        error('Joint axis must be +/- X ([0 0 1]), Y ([0 1 0]), or Z ([0 0 1])');
    end

end
