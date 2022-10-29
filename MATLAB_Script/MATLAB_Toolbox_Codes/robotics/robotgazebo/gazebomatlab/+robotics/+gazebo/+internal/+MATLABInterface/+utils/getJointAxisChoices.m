function axisChoice = getJointAxisChoices(modelname, jointname, op)
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo server plugin, calls get Gazebo
% model parameter which provides DOF( degree-of-freedom) of specific model
% joint.

%   Copyright 2020 The MathWorks, Inc.

    axisChoice = [];

    % Choices for Get axis parameter
    axisChoiceGet = ["Axis0","Axis1"];
    % Choices for Set axis parameter
    axisChoiceSet = ["0","1"];

    GazeboClient = robotics.internal.GazeboClient;

    % connect to Gazebo
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(GazeboClient);

    % get model-joint parameters
    modelParam = GazeboClient.getGazeboModelParam(modelname,false,jointname);

    % shutdown client
    GazeboClient.shutdown();

    % get degree of freedom ( Number of axis )
    jointIndex = modelParam.message.joints.dof;

    % return choices as per input operation ( 'set' or 'get' )
    if(strcmp(op,'set') && jointIndex > 0)
        axisChoice = axisChoiceSet(1:jointIndex);
    elseif(strcmp(op,'get') && jointIndex > 0)
        axisChoice = axisChoiceGet(1:jointIndex);
    end

end
