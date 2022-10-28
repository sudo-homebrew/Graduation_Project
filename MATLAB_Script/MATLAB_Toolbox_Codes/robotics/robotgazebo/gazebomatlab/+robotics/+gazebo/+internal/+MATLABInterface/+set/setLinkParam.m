function [status, errorMessage, printStatus, printMessage] = ...
        setLinkParam(modelName, linkName, p, varargin)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function creates MATLAB struct as per input model-link parameters.
    % Further, it connects with Gazebo and set model-link parameters of
    % corresponding Gazebo model.

    %   Copyright 2020 The MathWorks, Inc.

    % create model-link struct
    modelStruct.name = modelName;
    modelStruct.links = {};
    link = robotics.gazebo.internal.MATLABInterface.struct.getLinkParamStruct(linkName,p);
    modelStruct.links{end+1} = link;

    % connect to Gazebo, pass model-link struct, receive status and close
    % gazebo connection
    gazeboClient = robotics.internal.GazeboClient;
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    returnMessage = gazeboClient.setGazeboModelParam(modelStruct);
    gazeboClient.shutdown();

    % validate received message status
    robotics.gazebo.internal.MATLABInterface.utils.validateStatus(returnMessage.status, modelName, linkName, '');

    % get status and error message based on received message
    [status, errorMessage, printStatus, printMessage] = ....
        robotics.gazebo.internal.MATLABInterface.utils.getStatusMessage(...
            linkName, returnMessage, p, 'setLink', varargin);

end
