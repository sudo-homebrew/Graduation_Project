classdef IActionIntrospection < handle
    %This class is for internal use only. It may be removed in the future.
    
    %IActionIntrospection Interface for providing information about actions
    
    %   Copyright 2016-2020 The MathWorks, Inc.
    
    methods (Abstract)
        %actionList Get list of actions in ROS network
        namespaceList = actionList(obj)
        
        %actionType Get type of action in ROS namespace
        actType = actionType(obj, nameSpace)
        
        %actionInfo Get detailed information about action in ROS namespace
        infoStruct = actionInfo(obj, nameSpace)
        
        %infoStructToString Convert the action information structure into a string
        infoString = infoStructToString(obj, infoStruct)
    end
    
end
