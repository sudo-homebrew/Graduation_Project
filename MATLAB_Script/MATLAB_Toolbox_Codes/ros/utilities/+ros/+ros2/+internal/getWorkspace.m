function ret = getWorkspace(hObj)
% This function is for internal use only. It may be removed in the future.

% GETWORKSPACE is callback function to get the valid ROS 2 workspace folder
% saved in the model. If the folder saved in model does not exist, the
% function returns present working directory

% Copyright 2019 The MathWorks, Inc.

mdlName = get_param(hObj.getModel,'Name');
cfgSet = getActiveConfigSet(mdlName);
ret = pwd;
if isequal(get_param(mdlName,'HardwareBoard'), message('ros:slros2:codegen:ui_hwboard').getString)
    ctdata = codertarget.data.getData(cfgSet);
    if isfield(ctdata, 'ROS2Install') && isfield(ctdata.ROS2Install, 'Workspace') && isfolder(ctdata.ROS2Install.Workspace)
        ret = ctdata.ROS2Install.Workspace;
    end
end

end