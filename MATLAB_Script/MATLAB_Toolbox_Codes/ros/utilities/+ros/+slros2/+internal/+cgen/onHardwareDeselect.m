function onHardwareDeselect(hCS)
%This function is for internal use only. It may be removed in the future.

%ONHARDWAREDESELECT Executed when ROS 2 hardware is de-selected

%   See also ros.codertarget.internal.onHardwareSelect

%  Copyright 2019-2020 The MathWorks, Inc.
    ros.codertarget.internal.onHardwareDeselect(hCS);
    setProp(hCS, 'CodeInterfacePackaging', 'Nonreusable function');
end
