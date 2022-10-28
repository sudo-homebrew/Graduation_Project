function enableROSTimeStepping(hObj, hDlg, tag, desc)
%This function is for internal use only. It may be removed in the future.

%enableROSTimeStepping Called when user selects the ROS Time stepping checkbox

%   Copyright 2018 The MathWorks, Inc.

    isTimeStepping = logical(hDlg.getWidgetValue(tag));

    % Change the TLC file template
    if isTimeStepping
        tlcFile = 'rostime_codertarget_file_process.tlc';
    else
        tlcFile = 'codertarget_file_process.tlc';
    end
    set_param(bdroot, 'ERTCustomFileTemplate', tlcFile);

    % Execute the standard widget callback
    widgetChangedCallback(hObj, hDlg, tag, desc);

end
