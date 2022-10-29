classdef Util < ros.slros.internal.cgen.Util
    %This class is for internal use only. It may be removed in the future.
    
    %CGEN.UTIL - Utility functions for generating ROS2 C++ code
    
    % Copyright 2019-2021 The MathWorks, Inc.
    methods(Static)
        function [cppRosClass, msgType, headerName] = rosObjToCppClass(ros2Msg)
            msgType = ros2Msg.MessageType;
            if endsWith(msgType,'Request')
                srvType = 'Request';
                msgName = extractBefore(msgType,srvType);
                msgInfo = ros.internal.ros2.getServiceInfo(msgType,msgName,srvType);
            elseif endsWith(msgType,'Response')
                srvType = 'Response';
                msgName = extractBefore(msgType,srvType);
                msgInfo = ros.internal.ros2.getServiceInfo(msgType,msgName,srvType);
            else
                msgInfo = ros.internal.ros2.getMessageInfo(msgType);
            end
            cppRosClass = msgInfo.msgCppClassName;
            headerName = msgInfo.includeHeader;
        end       
        
        function ros2Header = rosCppClassToCppMsgHeader(cppClass)
            validateattributes(cppClass, {'char'}, {'nonempty'});
            assert(contains(cppClass, '::'));
            % Remove trailing ::Request, ::Response class name to derive
            % common header
            cppClass = regexprep(cppClass,'(\:\:Request)$','');
            cppClass = regexprep(cppClass,'(\:\:Response)$','');
            rosHeader = [strrep(cppClass,'::', '/') '.hpp'];
            [fpath,msgName,ext]=fileparts(rosHeader);
            hdrName = ros.internal.utilities.convertCamelcaseToLowercaseUnderscore(msgName);
            ros2Header = [fpath, '/',hdrName, ext];
        end
        
        function ret = getBusToCppMsgDefString(~)
            ret = '%s& msgPtr';
        end
        
        function ret = getCppToBusMsgDefString(~)
            ret = 'const %s& msgPtr';
        end        
        
        function ret = getSimpleAssignmentString(convertFromBus2Cpp, isROSEntity)
            % GETSIMPLEASSIGNMENTSTRING Get the assignment string for converting between buses and
            % CPP messages with respect to direction and nested messages
            if convertFromBus2Cpp    
                if isROSEntity
                    % Example: convertFromBus(msgPtr.position, &busPtr->position);
                    ret = '%s(%s.%s, &%s->%s);';
                else
                    % Example: msgPtr.nanosec =  busPtr->nanosec;
                    ret = '%s.%s = %s %s->%s;';
                end
            else % convertToBusFromCpp
                if isROSEntity
                    % Example: convertToBus(&busPtr->header, msgPtr.header);
                    ret = '%s(&%s->%s, %s.%s);';
                else
                    % Example: busPtr->nanosec =  msgPtr.nanosec;
                    ret = '%s->%s = %s %s.%s;';
                end
            end
        end
        
        function ret = getCopyArrayString(convertFromBusToCpp, isVarSize)
            if isVarSize
                if convertFromBusToCpp
                    ret = 'convertFromBusVariable%s(%s.%s, %s->%s, %s->%s);';
                    
                else
                    ret = 'convertToBusVariable%s(%s->%s, %s->%s, %s.%s, %s);';
                end
            else
                if convertFromBusToCpp
                    ret = 'convertFromBusFixed%s(%s.%s, %s->%s);';
                else
                    ret = 'convertToBusFixed%s(%s->%s, %s.%s, %s);';
                end
            end
        end
        
    end
end
