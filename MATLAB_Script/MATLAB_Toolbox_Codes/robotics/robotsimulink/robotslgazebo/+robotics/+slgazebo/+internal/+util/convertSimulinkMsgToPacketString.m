function [packetData] = convertSimulinkMsgToPacketString(cmd, msgType)
%This function is for internal use only. It may be removed in the future.
%       convertSimulinkMsgToPacketString calls importPacketField function
%       and converts Simulink message into MATLAB Struct of Packet string

%   Copyright 2020 The MathWorks, Inc.

    packetData = [];

    % retrieve packet string struct if importPacketField is available
    if(robotics.slgazebo.internal.util.checkCustomMessageOnPath('importPacketField.m'))
        packetData = importPacketField(cmd, msgType);
    end

end
