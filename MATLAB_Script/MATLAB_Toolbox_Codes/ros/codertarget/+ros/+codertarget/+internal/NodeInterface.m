classdef NodeInterface < handle
    %NodeInterface Abstract class defining ROS node interface

    % Copyright 2021 The MathWorks, Inc.
    methods (Access=public, Abstract)        
        runNode(obj,modelName,workspaceFolder,rosMasterURI,nodeHost,varargin);
        stopNode(obj,modelName);
        isRunning = isNodeRunning(obj,modelName);
        nodeList = getAvailableNodes(obj,exeFolder);
    end
end