function onBuildActionChange(~, hDlg, tag, ~)
%This function is for internal use only. It may be removed in the future.

% Copyright 2021 The MathWorks, Inc.
newVal = hDlg.getWidgetValue(tag);
ros.codertarget.internal.PropertyInfo.setgetBuildAction([],newVal);
end

