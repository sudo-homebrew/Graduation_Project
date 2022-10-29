function catkinPrefixPath = getCatkinPrefixPath
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

% checks the env variable and if none found returns the path to shipping
% dir

    envCatkinPath = getenv('CATKIN_PREFIX_PATH');
    if isempty(envCatkinPath)
        % MCR separates toolbox MATLAB-files from ROS packages
        % Use matlabroot to find correct path even in compiled code
        mlRoot = matlabroot;

        catkinPrefixPath = fullfile(mlRoot,'sys','ros1',computer('arch'),'ros1');
    else
        catkinPrefixPath = envCatkinPath;
        if startsWith(catkinPrefixPath,'"') && endsWith(catkinPrefixPath,'"')
            %remove enclosing "
            catkinPrefixPath = catkinPrefixPath(2:end-1);
        end
    end
