function msgSpec = getMsgSpec(messageType, msgPath)
%This function is for internal use only. It may be removed in the future.

%GETMSGSPEC Returns message spec for a given message type.

%   Copyright 2020 The MathWorks, Inc.

[msgPkgName,msgFileName] = fileparts(messageType);
builtinMsgPath = fullfile(matlabroot,'sys','ros1',computer('arch'),'ros1','share');
customMsgPath = fullfile(matlabroot,'toolbox','ros','mlroscpp',...
    'custom_messages','share');
reg = ros.internal.CustomMessageRegistry.getInstance('ros');
customMsgInfo = reg.getMessageInfo(messageType);
if isempty(customMsgInfo)
    location = ros.internal.utilities.locateMessage(msgPkgName,msgFileName,{msgPath,builtinMsgPath,customMsgPath},'msg');
else
    location = customMsgInfo.srcPath;
end
location = fileparts(fileparts(fileparts(location))); % Find top level directory where msgPkgName is located

map = ros.internal.utilities.getSpecialMapForROS;
parser = ros.internal.MessageParser(messageType,{location,builtinMsgPath,customMsgPath},map);

msgDefn = getMessageDefinition(parser);
msgDefn.msgInfo = ros.internal.ros.getMessageInfo(messageType, reg);
msgDefn = ros.internal.ros.augmentMessageDefinition(msgDefn);
msgSpec = ros.internal.ros.createMsgSpec(msgDefn);
end

