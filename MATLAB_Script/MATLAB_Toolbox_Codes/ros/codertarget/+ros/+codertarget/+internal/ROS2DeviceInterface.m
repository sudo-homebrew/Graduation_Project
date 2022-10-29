classdef (Abstract) ROS2DeviceInterface < handle
    %ROS2DeviceInterface Abstract class defining device interface for a 
    % ROS 2 device

    % Copyright 2021 The MathWorks, Inc.
    methods (Access=public, Abstract)
        % System interface
        output = system(obj,command,sudo);
        putFile(obj,localFile,remoteFile);
        getFile(obj,remoteFile,localFile);
        deleteFile(obj,fileName);
        d = dir(obj,fileSpec);
        
        % Node interface
        runNode(obj, modelName, rosMasterURI, nodeHost);
        stopNode(obj, modelName);
        isRunning = isNodeRunning(obj, modelName);
    end
end