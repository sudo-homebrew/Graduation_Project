classdef (Abstract) ROSDeviceInterface < handle
    %ROSDeviceInterface Abstract class defining device interface for a ROS
    % device

    % Copyright 2021 The MathWorks, Inc.
    methods (Access=public, Abstract)
        % System interface
        output = system(obj,command,sudo);
        putFile(obj,localFile,remoteFile);
        getFile(obj,remoteFile,localFile);
        deleteFile(obj,fileName);
        d = dir(obj,fileSpec);
        
        % Core interface
        isRunning = isCoreRunning(obj);
        stopCore(obj);
        runCore(obj);
        
        % Node interface
        runNode(obj, modelName, rosMasterURI, nodeHost);
        stopNode(obj, modelName);
        isRunning = isNodeRunning(obj, modelName);
    end
end