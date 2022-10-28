function modelList = getModelList
%This function is for internal use only. It may be removed in the future.
%
% This function connects calls Gazebo model entity which provides
% Gazebo model details. Further, it returns model name list.

%   Copyright 2020 The MathWorks, Inc.

    modelInfo = robotics.gazebo.internal.MATLABInterface.utils.getEntityList;

    modelList = [modelInfo.model_data.model_name];

end
