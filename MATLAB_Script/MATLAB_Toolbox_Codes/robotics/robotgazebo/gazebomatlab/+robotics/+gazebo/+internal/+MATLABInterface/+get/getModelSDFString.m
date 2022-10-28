function sdfString = getModelSDFString(modelName)
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo and request Gazebo model SDF string
% based on modelname.

%   Copyright 2021 The MathWorks, Inc.

% connect with Gazebo and get SDF details
    gazeboClient = robotics.internal.GazeboClient;
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient)
    sdfDetails = gazeboClient.getGazeboModelSDF(modelName);

    % error-out if requested model is not available in Gazebo
    if(strcmp(sdfDetails.message.model_name,"") || ...
       ~strcmp(sdfDetails.message.model_name,modelName))
        error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                      modelName,'gzmodel("list")'));
    end

    % get SDF string
    sdfString = sdfDetails.message.sdf_string;

    gazeboClient.shutdown();

end
