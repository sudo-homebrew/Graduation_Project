function amentPrefixPath = getAmentPrefixPath
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2021 The MathWorks, Inc.

% Always use MATLAB's Ament installation, not user's environment variable

% MCR separates toolbox MATLAB-files from ROS packages
% Use matlabroot to find correct path even in compiled code
mlRoot = matlabroot;

amentPrefixPath = fullfile(mlRoot,'sys','ros2',computer('arch'),'ros2');
