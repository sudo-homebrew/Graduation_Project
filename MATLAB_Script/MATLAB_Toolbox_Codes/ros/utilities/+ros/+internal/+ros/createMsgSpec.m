function msgSpec = createMsgSpec(msgDefn)
%This function is for internal use only. It may be removed in the future.

%CREATEMSGSPEC Creates a message spec for use with computeMsgMD5 function.

%   Copyright 2020 The MathWorks, Inc.

msgSpec = struct;
if isfield(msgDefn,'filePath')
    msgSpec.FilePath = msgDefn.filePath;
end
for k = 1:numel(msgDefn.fieldNames)
    name = msgDefn.fieldNames{k};
    msgSpec.Field(k).Name = name;
    if isfield(msgDefn.msgFields.(name),'ROSdataType')
        msgSpec.Field(k).Type = msgDefn.msgFields.(name).ROSdataType;
    else
        msgSpec.Field(k).Type = msgDefn.msgFields.(name).MessageType;
    end
    msgSpec.Field(k).ArraySize = msgDefn.msgFields.(name).count;
    msgSpec.Field(k).IsConstant = ~(isscalar(msgDefn.msgFields.(name).constantValue) ...
        && isnumeric(msgDefn.msgFields.(name).constantValue) ...
        && isnan(msgDefn.msgFields.(name).constantValue));
    if msgSpec.Field(k).IsConstant
        msgSpec.Field(k).Value = msgDefn.msgFields.(name).rawConstantValue;
    else
        msgSpec.Field(k).Value = NaN;
    end
end
end