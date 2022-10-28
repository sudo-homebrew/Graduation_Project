function [boostRootPath, boostIncludePath] = getBoostRootPath
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2020 The MathWorks, Inc.

% checks the env variable and if none found returns the path to shipping
% dir

    envBootRootPath = getenv('BOOST_ROOT');
    if isempty(envBootRootPath)
        % MCR separates toolbox MATLAB-files from ROS packages
        % Use matlabroot to find correct path even in compiled code
        mlRoot = matlabroot;
        boostRootPath = fullfile(mlRoot,'sys','ros1',computer('arch'),'ros1','boost');
    else
        boostRootPath = envBootRootPath;
        if startsWith(boostRootPath,'"') && endsWith(boostRootPath,'"')
            %remove enclosing "
            boostRootPath = boostRootPath(2:end-1);
        end
    end
    boostIncludePath = fullfile(boostRootPath,'include');
    if ~isfolder(fullfile(boostIncludePath,'boost'))
        %most probably we have versions
        %pick the first one and look inside
        dirlist = dir(boostIncludePath);
        dirlist = dirlist(arrayfun(@(x)~isequal(x.name,'.')&&~isequal(x.name,'..'),dirlist));
        boostIncludePath = fullfile(dirlist(1).folder,dirlist(1).name);
        boostPath = fullfile(boostIncludePath,'boost');
        assert(isfolder(boostPath),message('MATLAB:cd:NonExistentFolder',boostPath)); %should have boost there
    end
