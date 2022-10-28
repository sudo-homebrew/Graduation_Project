function out = convertMsgClassToStruct(in)
%This function is for internal use only. It may be removed in the future.

%convertMsgClassToStruct automatically convert Gazebo_msgs classes into
%corresponding MATLAB structure that matches the bus derived from these
%classes.

%   Copyright 2019 The MathWorks, Inc.

out = robotics.slgazebo.internal.util.convertStringToUint8(in);
out = removeMessageType(out);
out = robotics.slgazebo.internal.util.padSignals(out);

end

function y = removeMessageType(x)
%removeMessageType removes 'MessageType' field from nested structs

    y = rmfield(x, 'MessageType');
    fieldNameToProcess = fieldnames(y);
    for idx = 1:numel(fieldNameToProcess)
        if isstruct(y.(fieldNameToProcess{idx}))
            y.(fieldNameToProcess{idx}) = removeMessageType(y.(fieldNameToProcess{idx}));
        end
    end

end

