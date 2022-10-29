function setWorkspace(hObj, hDlg, tag, desc)
% This function is for internal use only. It may be removed in the future.

% SETWORKSPACE is callback function when user changes the ROS 2 workspace
% folder from Hardware implementation Simulink Configuration parameters.

% Copyright 2019 The MathWorks, Inc.

newVal = hDlg.getWidgetValue(tag);
validateattributes(newVal,{'char'},{'nonempty','row'},'','Username');
if ~isfolder(newVal)
    error(message('ros:slros2:codegen:InvalidWorkspaceFolder',newVal));
end
hDlg.setWidgetValue(tag, newVal);
widgetChangedCallback(hObj, hDlg, tag, desc);
codertarget.data.setParameterValue(hObj, 'ROS2Install.Workspace', newVal);
end
