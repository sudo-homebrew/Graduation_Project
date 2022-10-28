function [status, errorMessage, printStatus, printMessage] = ...
        setModelParam(modelName, p, varargin)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function creates MATLAB struct as per input model parameters.
    % Further, it connects with Gazebo and set model parameters of
    % corresponding Gazebo model.

    %   Copyright 2020 The MathWorks, Inc.

    % create model struct
    modelStruct.name = modelName;
    modelStruct = robotics.gazebo.internal.MATLABInterface.struct.getModelParamStruct(modelStruct,p);

    % connect to Gazebo, pass model struct, receive status and close gazebo
    % connection
    gazeboClient = robotics.internal.GazeboClient;
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    returnMessage = gazeboClient.setGazeboModelParam(modelStruct);
    gazeboClient.shutdown();

    % validate received message status
    robotics.gazebo.internal.MATLABInterface.utils.validateStatus(returnMessage.status, modelName, '', '');

    % get status and error message based on received message
    [status, errorMessage, printStatus, printMessage] = ....
        robotics.gazebo.internal.MATLABInterface.utils.getStatusMessage(...
            '', returnMessage, p, 'setModel', varargin);

end
