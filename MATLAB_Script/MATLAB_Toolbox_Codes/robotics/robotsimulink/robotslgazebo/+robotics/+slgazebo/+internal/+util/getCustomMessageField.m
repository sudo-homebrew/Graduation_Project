function [result] = getCustomMessageField(customData,topicType)
%This function is for internal use only. It may be removed in the future.
%       getCustomMessageField calls importCustomMsgField function
%       and retrieve custom message fields into MATLAB Struct from
%       received Packet struct

%   Copyright 2020 The MathWorks, Inc.

    result =[];

    % retrieve MATLAB Struct if importCustomMsgField is available
    if(robotics.slgazebo.internal.util.checkCustomMessageOnPath('importCustomMsgField.m'))
        result = importCustomMsgField(customData,topicType);
    end

end
