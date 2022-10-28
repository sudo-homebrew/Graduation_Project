function rootDir = getRootDir
%This function is for internal use only. It may be removed in the future.

%getRootDir Return the root directory of Simulink ROS functionality
%   This is the directory that has registry/ for Coder Target
%   and the src/ subdirectories.

%   Copyright 2014-2018 The MathWorks, Inc.

    rootDir = fullfile(matlabroot, 'toolbox', 'ros', 'slros');

end
