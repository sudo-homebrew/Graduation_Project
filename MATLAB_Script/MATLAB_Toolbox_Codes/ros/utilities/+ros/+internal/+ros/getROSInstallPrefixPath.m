function ROSInstallPrefixPath = getROSInstallPrefixPath
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

% checks the env variable and if none found returns the path to shipping
% dir

    envCatkinPath = getenv('CATKIN_PREFIX_PATH');
    if isempty(envCatkinPath)
        % MCR separates toolbox MATLAB-files from ROS packages
        % Use matlabroot to find correct path even in compiled code
        mlRoot = matlabroot;

        ROSInstallPrefixPath = fullfile(mlRoot,'sys','ros1',computer('arch'),'ros1');
    else
        ROSInstallPrefixPath = envCatkinPath;
        if startsWith(ROSInstallPrefixPath,'"') && endsWith(ROSInstallPrefixPath,'"')
            %remove enclosing "
            ROSInstallPrefixPath = ROSInstallPrefixPath(2:end-1);
        end
    end
