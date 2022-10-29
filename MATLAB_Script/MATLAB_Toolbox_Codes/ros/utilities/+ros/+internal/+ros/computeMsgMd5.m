function md5Value = computeMsgMd5(msgSpec, refMsgTypeCheckSumMap, msgDefn)
%This function is for internal use only. It may be removed in the future.

%COMPUTEMSGMD5 Computes MD5 checksum of a message.

%   Copyright 2020 The MathWorks, Inc.

% Steps to compute MD5 hash of a message:
% * Remove comments and insignificant whitespaces
% * Remove package names of dependencies
% * Re-order constants ahead of other declarations
% * Replace dependencies by MD5 text
%
% Ref: https://github.com/ros/genmsg/blob/42e364661025b8f3d51486dc513d9e29cbd308da/src/genmsg/gentools.py#L59-L90
buff = "";
if ~isempty(msgSpec) && isfield(msgSpec,'Field')
    % Re-order constants ahead of other declarations
    for k = 1:numel(msgSpec.Field)
        if msgSpec.Field(k).IsConstant
            c = msgSpec.Field(k);
            buff = buff + sprintf("%s %s=%s\n", c.Type, c.Name, string(c.Value));
        end
    end
    
    % Process components of the message fields
    for k = 1:numel(msgSpec.Field)
        if msgSpec.Field(k).IsConstant
            % Constants are re-ordered ahead of other declarations to
            % compute MD5 hash. They are already processed at this point.
            continue;
        end
        f = msgSpec.Field(k);
        if isBuiltinType(f.Type)
            if ismember(f.Type,{'ros/Time','ros/Duration'})
                [~,msgType] = fileparts(f.Type);
                msgType = lower(msgType);
            else
                msgType = f.Type;
            end
            if isnan(f.ArraySize)
                buff = buff + sprintf("%s[] %s\n", msgType, f.Name);
            else
                if f.ArraySize > 0
                    buff = buff + sprintf("%s[%d] %s\n", msgType, f.ArraySize, f.Name);
                else
                    buff = buff + sprintf("%s %s\n", msgType, f.Name);
                end
            end
        else
            % Recursively generate md5 for referenced type
            if ~isKey(refMsgTypeCheckSumMap, f.Type)
                refMsgSpec = ros.internal.ros.createMsgSpec(msgDefn.msgFields.(f.Name));
                refMd5 = ros.internal.ros.computeMsgMd5(refMsgSpec, refMsgTypeCheckSumMap, msgDefn.msgFields.(f.Name));
                refMsgTypeCheckSumMap(f.Type) = refMd5;
            else
                refMd5 = refMsgTypeCheckSumMap(f.Type);
            end
            % Replace referenced type by its checksum. Drop array
            % qualifiers.
            buff = buff + sprintf("%s %s\n", refMd5, f.Name);
        end
    end
end
buff = strtrim(buff); % Remove trailing \n
md5Value = ros.internal.utilities.md5str(buff);
end

function ret = isBuiltinType(msgType)
% According to https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/#field-types
% The following are the built-in message types:
BUILTIN_TYPE = {'int8','uint8','int16','uint16','int32','uint32',...
    'int64','uint64','float32','float64','string','bool', ...
    'char','byte','ros/Time','ros/Duration', 'wstring'};

ret = ismember(msgType, BUILTIN_TYPE);
end
