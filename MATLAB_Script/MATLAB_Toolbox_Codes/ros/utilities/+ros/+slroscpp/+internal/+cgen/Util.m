classdef Util < ros.slros.internal.cgen.Util
    %This class is for internal use only. It may be removed in the future.
    
    %CGEN.UTIL - Utility functions for generating ROS C++ code
    
    % Copyright 2020 The MathWorks, Inc.
    methods(Static)
        function [cppRosClass, msgType, headerName] = rosObjToCppClass(rosMsg)
            timeMsgType = ros.slros.internal.bus.Util.TimeMessageType;
            durationMsgType = ros.slros.internal.bus.Util.DurationMessageType;
            if isequal(rosMsg.MessageType,'ros/Time') || ...
                    isequal(rosMsg.MessageType,timeMsgType)
                msgType = timeMsgType;
                msgInfo = ros.internal.ros.getMessageInfo('ros/Time');
                cppRosClass = msgInfo.msgCppClassName;
                headerName = msgInfo.includeHeader;
            elseif isequal(rosMsg.MessageType,'ros/Duration') || ...
                    isequal(rosMsg.MessageType,durationMsgType)
                msgType = durationMsgType;
                msgInfo = ros.internal.ros.getMessageInfo('ros/Duration');
                cppRosClass = msgInfo.msgCppClassName;
                headerName = msgInfo.includeHeader;
            else
                msgType = rosMsg.MessageType;
                msgInfo = ros.internal.ros.getMessageInfo(rosMsg.MessageType);
                cppRosClass = msgInfo.msgCppClassName;
                headerName = msgInfo.includeHeader;
            end
        end
        
        function header = rosCppClassToCppMsgHeader(cppClass)
            validateattributes(cppClass, {'char'}, {'nonempty'});
            assert(contains(cppClass, '::'));
            header = [strrep(cppClass,'::', '/') '.h'];
        end   

        function [propList, rosPropList] = getROSMsgPropertyList(emptyRosMsg)
            propList = setdiff(fieldnames(emptyRosMsg), 'MessageType');
            rosPropList = propList;
        end        
    end
end
