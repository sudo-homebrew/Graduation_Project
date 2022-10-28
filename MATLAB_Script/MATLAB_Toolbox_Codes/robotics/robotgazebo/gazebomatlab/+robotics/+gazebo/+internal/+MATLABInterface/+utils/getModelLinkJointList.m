function modelLinkJointList = getModelLinkJointList(modelname, op)
%This function is for internal use only. It may be removed in the future.
%
% This function connects calls Gazebo model entity which provides
% Gazebo model details. Further, it returns model-links or model-joints
% name list.

%   Copyright 2020 The MathWorks, Inc.

    modelInfo = robotics.gazebo.internal.MATLABInterface.utils.getEntityList;

    linkJointId = strcmp(modelname,[modelInfo.model_data.model_name]);
    modelData = modelInfo.model_data(linkJointId);

    if(strcmp(op,'link'))
        modelLinkJointList = modelData.links.link_name;
    else
        modelLinkJointList = modelData.joints.joint_name;
    end

end
