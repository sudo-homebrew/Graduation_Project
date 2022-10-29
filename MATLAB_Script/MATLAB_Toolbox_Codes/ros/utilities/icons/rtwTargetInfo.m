function rtwTargetInfo(~)
% This function is for internal use only. It may be removed in the future.

%RTWTARGETINFO Register icons and refresh library dictionary

%   Copyright 2020-2022 The MathWorks, Inc.

    if ~isempty(ver('simulink')) % if Simulink is installed
        % Register block icons
        ros.internal.registerIcons();
    end
end

