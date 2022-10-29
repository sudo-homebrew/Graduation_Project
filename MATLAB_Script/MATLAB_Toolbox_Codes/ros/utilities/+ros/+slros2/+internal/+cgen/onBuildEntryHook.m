function onBuildEntryHook(hCS)
%This function is for internal use only. It may be removed in the future.

%ONBUILDENTRYHOOK Entry hook point for ROS2 code generation

%   Copyright 2019 The MathWorks, Inc.

% Check that target language is C++ and error out if it is not

ros.codertarget.internal.onBuildEntryHook(hCS);
end