function [status, errorMessage, printStatus, printMessage] = ...
        setJointParam( modelName, jointName, p, varargin)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function creates MATLAB struct as per input model-joint parameters.
    % Further, it connects with Gazebo and set model-joint parameters of
    % corresponding Gazebo model.

    %   Copyright 2020 The MathWorks, Inc.

    % create model-joint struct
    modelStruct.name = modelName;
    modelStruct.joints = {};
    joint = robotics.gazebo.internal.MATLABInterface.struct.getJointParamStruct(jointName,p);
    modelStruct.joints{end+1} = joint;

    % connect to Gazebo, pass model-joint struct, receive status and close
    % gazebo connection
    gazeboClient = robotics.internal.GazeboClient;
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    returnMessage = gazeboClient.setGazeboModelParam(modelStruct);
    gazeboClient.shutdown();

    % validate received message status
    robotics.gazebo.internal.MATLABInterface.utils.validateStatus(....
        returnMessage.status, modelName, jointName, p.Results.Axis);

    % get status and error message based on received message
    [status, errorMessage, printStatus, printMessage] = ....
        robotics.gazebo.internal.MATLABInterface.utils.getStatusMessage(...
            jointName, returnMessage, p, 'setJoint', varargin);

end
